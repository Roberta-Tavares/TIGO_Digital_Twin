#!/usr/bin/env bash
# Inicia a simulação do TIGO no Ignition Gazebo já despausada (-r).
# Passe um arquivo SDF para escolher outro mundo; por padrão, usa o LACEA.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"
WORLD="${1:-worlds/lacea.sdf}"

if [[ ! -f "$WORLD" ]]; then
  echo "Arquivo de mundo não encontrado: $WORLD" >&2
  exit 1
fi

echo "Iniciando Ignition Gazebo com $WORLD..."
ign gazebo -r "$WORLD"
