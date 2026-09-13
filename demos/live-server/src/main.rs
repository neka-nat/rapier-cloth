//! Local, demand-driven demo transport. Each connection owns one simulation.
mod simulation;
use axum::{
    Router,
    extract::{
        State, WebSocketUpgrade,
        ws::{Message, WebSocket},
    },
    http::{HeaderMap, StatusCode},
    response::{IntoResponse, Response},
    routing::get,
};
use serde::Deserialize;
use simulation::{Command, Demo, SceneKind};
use std::{sync::Arc, time::Duration};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Request {
    request_id: u32,
    command: Command,
}

async fn upgrade(
    State(origins): State<Arc<Vec<String>>>,
    headers: HeaderMap,
    ws: WebSocketUpgrade,
) -> Response {
    if !headers
        .get("origin")
        .and_then(|h| h.to_str().ok())
        .is_some_and(|origin| origins.iter().any(|allowed| origin == allowed))
    {
        return (StatusCode::FORBIDDEN, "Origin is not allowed").into_response();
    }
    ws.max_message_size(2048)
        .max_frame_size(2048)
        .write_buffer_size(0)
        .max_write_buffer_size(256 * 1024)
        .on_upgrade(connection)
}
async fn send(socket: &mut WebSocket, value: &impl serde::Serialize) -> bool {
    let Ok(text) = serde_json::to_string(value) else {
        return false;
    };
    matches!(
        tokio::time::timeout(
            Duration::from_secs(10),
            socket.send(Message::Text(text.into()))
        )
        .await,
        Ok(Ok(()))
    )
}
async fn connection(mut socket: WebSocket) {
    let Ok(Ok(mut demo)) = tokio::task::spawn_blocking(|| Demo::new(SceneKind::Drape)).await else {
        return;
    };
    if !send(&mut socket, &demo.frame(0, true)).await {
        return;
    }
    let mut last_request = 0;
    while let Some(Ok(message)) = socket.recv().await {
        let text = match message {
            Message::Text(text) => text,
            Message::Close(_) => break,
            _ => continue,
        };
        let request = match serde_json::from_str::<Request>(&text) {
            Ok(request) if request.request_id > last_request => request,
            _ => {
                send(&mut socket, &serde_json::json!({"type":"error","request_id":0,"message":"不正な操作です。再接続してください。"})).await;
                break;
            }
        };
        last_request = request.request_id;
        // Only four bounded substeps run on the blocking pool per request.
        // Await delivery before reading another command: no background producer
        // or unbounded queue, including while a browser is paused/disconnected.
        let Ok((next, result)) = tokio::task::spawn_blocking(move || {
            let result = demo.command(request.command, request.request_id);
            (demo, result)
        })
        .await
        else {
            break;
        };
        demo = next;
        let sent = match result {
            Ok(frame) => send(&mut socket, &frame).await,
            Err(message) => send(
                &mut socket,
                &serde_json::json!({"type":"error","request_id":last_request,"message":message}),
            )
            .await,
        };
        if !sent {
            break;
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut port: u16 = 9100;
    let mut origins = vec![
        "http://127.0.0.1:5173".into(),
        "http://localhost:5173".into(),
    ];
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--port" => port = args.next().ok_or("--port needs a value")?.parse()?,
            "--origin" => origins.push(args.next().ok_or("--origin needs a value")?),
            "--help" => {
                println!("rapier-cloth-live [--port 9100] [--origin http://127.0.0.1:5173]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    let app = Router::new()
        .route(
            "/health",
            get(|| async {
                axum::Json(serde_json::json!({"name":"rapier-cloth-live","protocol":1}))
            }),
        )
        .route("/live/ws", get(upgrade))
        .with_state(Arc::new(origins));
    let listener = tokio::net::TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, port)).await?;
    println!("Cloth CPU server: http://{}", listener.local_addr()?);
    axum::serve(listener, app).await?;
    Ok(())
}
