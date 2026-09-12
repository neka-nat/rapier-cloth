#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
task_log=$(mktemp)
trap 'rm -f "$task_log"' EXIT
for task_crate in rapier-cloth-core rapier-cloth; do
    for task_features in none both; do
        task_args=(check --locked -p "$task_crate" --no-default-features)
        if [[ "$task_features" == both ]]; then
            task_args+=(--features f32,f64)
        fi
        if cargo "${task_args[@]}" >"$task_log" 2>&1; then
            cat "$task_log"
            echo "Invalid precision selection unexpectedly succeeded: $task_crate $task_features" >&2
            exit 1
        fi
        if ! rg -q 'select exactly one precision feature: f32 or f64' "$task_log"; then
            cat "$task_log"
            exit 1
        fi
    done
done
echo 'Invalid precision combinations rejected with the expected diagnostic.'
