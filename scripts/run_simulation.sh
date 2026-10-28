#!/usr/bin/env bash
# Inicia a simulação do TIGO no Ignition Gazebo já despausada (-r)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"
echo "Iniciando Ignition Gazebo com worlds/lacea.sdf..."
ign gazebo -r worlds/lacea.sdf
