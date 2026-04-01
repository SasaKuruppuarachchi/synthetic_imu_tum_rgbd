#!/usr/bin/env bash
set -euo pipefail

# ================= FLAGS =================
REMOVE_SOURCE=false
POSITIONAL=()
for _arg in "$@"; do
  case "$_arg" in
    --remove-source) REMOVE_SOURCE=true ;;
    *) POSITIONAL+=("$_arg") ;;
  esac
done
set -- "${POSITIONAL[@]+"${POSITIONAL[@]}"}"

# ================= CONFIG =================
INPUT_DIR="${1:-.}"
OUTPUT_ROOT="${2:-./converted}"

TMP_DIR="${OUTPUT_ROOT}/tmp"
MCAP_DIR="${OUTPUT_ROOT}/mcap"
LOG_DIR="${OUTPUT_ROOT}/logs"

mkdir -p "$TMP_DIR" "$MCAP_DIR" "$LOG_DIR"

LOG_FILE="${LOG_DIR}/conversion_$(date +%Y%m%d_%H%M%S).log"

# ================= LOGGING =================
log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

error() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: $*" | tee -a "$LOG_FILE" >&2
}

# ================= CHECKS =================
command -v rosbags-convert >/dev/null || { error "rosbags-convert not found"; exit 1; }
command -v ros2 >/dev/null || { error "ros2 not found (source ROS2)"; exit 1; }

# optional pv
if command -v pv >/dev/null; then
  USE_PV=true
else
  USE_PV=false
fi

# ================= DISCOVER FILES =================
mapfile -t BAGS < <(find "$INPUT_DIR" -name "*.bag" | sort)
TOTAL=${#BAGS[@]}

[[ "$TOTAL" -eq 0 ]] && { error "No .bag files found"; exit 1; }

log "Found $TOTAL bag files"

SUCCESS=0
FAIL=0
SKIP=0
COUNT=0

# ================= MAIN LOOP =================
for BAG in "${BAGS[@]}"; do
  COUNT=$((COUNT + 1))
  BASENAME=$(basename "$BAG" .bag)

  ROS2_DIR="${TMP_DIR}/${BASENAME}_ros2"
  MCAP_OUT="${MCAP_DIR}/${BASENAME}_mcap"
  YAML_FILE="${TMP_DIR}/${BASENAME}_convert.yaml"

  log "========================================"
  log "[$COUNT/$TOTAL] Processing: $BASENAME"

  # ===== RESUME =====
  if [[ -d "$MCAP_OUT" ]] && find "$MCAP_OUT" -name "*.mcap*" | grep -q .; then
    log "[skip] Already converted"
    SKIP=$((SKIP + 1))
    continue
  fi

  # ===== STEP 1: ROS1 -> ROS2 =====
  log "[1/2] ROS1 -> ROS2"

  rm -rf "$ROS2_DIR"

  if $USE_PV; then
    SIZE=$(du -sb "$BAG" | cut -f1)
    pv -s "$SIZE" "$BAG" > /dev/null
  else
    log "[progress] $(du -h "$BAG" | cut -f1)"
  fi

  if ! rosbags-convert --src "$BAG" --dst "$ROS2_DIR" >>"$LOG_FILE" 2>&1; then
    error "ROS1 -> ROS2 failed"
    FAIL=$((FAIL + 1))
    continue
  fi

  log "[1/2] Done"

  # ===== FIX METADATA (IMPORTANT STEP) =====
  log "[fix] Patching metadata.yaml offered_qos_profiles"

  METADATA_FILE=$(find "$ROS2_DIR" -name "metadata.yaml" -print -quit)

  if [[ -n "$METADATA_FILE" && -f "$METADATA_FILE" ]]; then
    sed -i 's/offered_qos_profiles: \[\]/offered_qos_profiles: null/g' "$METADATA_FILE"
    log "[fix] metadata.yaml patched: $METADATA_FILE"
  else
    error "metadata.yaml not found, skipping patch step"
  fi

  # ===== STEP 2: ROS2 -> MCAP (CORRECT YAML) =====
  log "[2/2] ROS2 -> MCAP"

  rm -rf "$MCAP_OUT"

  cat > "$YAML_FILE" <<EOF
output_bags:
  - uri: ${MCAP_OUT}
    storage_id: mcap
    compression_mode: file
    compression_format: zstd
EOF

  if ! ros2 bag convert -i "$ROS2_DIR" -o "$YAML_FILE" >>"$LOG_FILE" 2>&1; then
    error "ROS2 -> MCAP failed"
    FAIL=$((FAIL + 1))
    continue
  fi

  # ===== VERIFY OUTPUT =====
  if ! find "$MCAP_OUT" -name "*.mcap*" | grep -q .; then
    error "No MCAP produced"
    FAIL=$((FAIL + 1))
    continue
  fi

  log "[2/2] Done"

  # ===== CLEANUP =====
  rm -rf "$ROS2_DIR" "$YAML_FILE"

  # ===== REMOVE SOURCE (optional) =====
  if [[ "$REMOVE_SOURCE" == "true" ]]; then
    rm -f "$BAG"
    log "[removed] Source: $BAG"
  fi

  SUCCESS=$((SUCCESS + 1))
  log "[✓] Completed"

done

# ================= SUMMARY =================
log "========================================"
log "Total:    $TOTAL"
log "Success:  $SUCCESS"
log "Skipped:  $SKIP"
log "Failed:   $FAIL"
log "Output:   $MCAP_DIR"
log "Log file: $LOG_FILE"
