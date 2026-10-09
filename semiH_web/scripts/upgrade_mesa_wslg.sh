#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# WSLg GPU 렌더 진단 스크립트 (2026-08-29 개정)
#
# 배경: 웹 UI에서 mode 를 켜면 Gazebo GUI 창이 화면에 안 뜨고 작업표시줄
# 아이콘만 남는다("copy mode" 커서). 원인을 추적한 결과 **Mesa 버전 문제가
# 아니라 WSL2 ↔ Windows 사이의 GPU 가상화(GPU-PV) 채널이 깨져 있는 것**으로
# 확인됐다:
#
#   - /dev/dri 렌더 노드가 아예 생성되지 않음
#   - dmesg 에 `misc dxg: dxgk: dxgkio_query_adapter_info: Ioctl failed: -22/-2`
#     (dxg 커널 드라이버가 Windows 호스트 GPU 커널과 통신 실패)
#   - /mnt/wslg/stderr.log 에 `Xwayland glamor ... falling back to sw`
#   - 결과적으로 rviz2/gazebo/glxinfo 전부 llvmpipe(소프트웨어)로만 동작
#   - NVIDIA RTX 5050(Blackwell) + Windows 드라이버 596.13 인데
#     /usr/lib/wsl/lib/libd3d12core.so 는 2023-10-20 빌드(너무 오래됨),
#     WSL 쪽 NVIDIA 런타임은 595.68 (Windows 596.13 과 불일치)
#
# => 이 계층은 apt/Mesa 로 못 고친다. **Windows 쪽 조치가 필요**하다:
#
#   1) Windows PowerShell(관리자):
#         wsl --update            # WSL 커널 + /usr/lib/wsl/lib/* 런타임 갱신
#         wsl --version           # 갱신 확인
#      (Store 버전이 아니면: `wsl --update --web-download`)
#
#   2) Windows NVIDIA 드라이버를 최신으로 "클린 설치"
#      (RTX 5050 = Blackwell, 최신 Game Ready/Studio 드라이버 필요).
#      nvidia-smi 의 "Driver Version" 과 "NVIDIA-SMI" 버전이 맞아야 정상.
#
#   3) Windows PowerShell:  wsl --shutdown   후 WSL 터미널 새로 열기
#
#   4) 아래 진단을 다시 실행해 [OK] 가 뜨는지 확인:
#         bash scripts/upgrade_mesa_wslg.sh
#
#   5) GPU 렌더가 살아나면(=/dev/dri 생성, glxinfo renderer 가 "D3D12 (NVIDIA
#      GeForce RTX 5050)") :
#         sudo apt install qtwayland5
#      그리고 config/settings.yaml 의 gui_env 를 주석대로
#         QT_QPA_PLATFORM: "wayland;xcb"
#      한 줄로 교체 → streamlit 완전 재시작 → 환경 카드의
#      "Gazebo GUI 창도 띄우기" 체크박스 켜고 mode 시작.
#
# 그 전까지는 웹 UI 기본값(headless) + "RViz 열기" 로 시각화하면 된다
# (rviz2 는 소프트웨어 렌더로도 창이 정상적으로 뜬다 — 검증됨).
#
# --- (참고) 예전 계획이 왜 폐기됐나 -------------------------------------
# 이전 세션은 `ppa:kisak/kisak-mesa` 로 Mesa 를 올리는 방법을 적어놨지만,
# 그 PPA 의 jammy(22.04) 저장소는 현재 **비어 있다**(InRelease 안의 Packages
# 인덱스가 0바이트). 그래서 `add-apt-repository` + `apt update` 를 해도
# `apt-cache policy` 에 새 버전이 안 뜨고 "All packages are up to date" 만
# 나온다. 설령 Mesa 를 올려도 위 [2][3] 커널 계층 문제는 안 고쳐진다.
# 굳이 최신 Mesa 를 원하면 아직 유지되는 `ppa:oibaf/graphics-drivers` 가
# 대안이지만, 이 머신의 근본 원인(GPU-PV 채널)과는 무관하다.
# ---------------------------------------------------------------------------
set -uo pipefail

pass=0; fail=0
ok()   { echo "  [OK]   $*"; pass=$((pass+1)); }
bad()  { echo "  [FAIL] $*"; fail=$((fail+1)); }
info() { echo "  [ .. ] $*"; }

echo "== 1. GPU 렌더 노드 =="
if [[ -e /dev/dri/renderD128 ]]; then ok "/dev/dri/renderD128 존재"
else bad "/dev/dri 렌더 노드 없음 → GPU-PV 미초기화 (wsl --update 필요)"; fi

echo "== 2. dxg 커널 ↔ 호스트 GPU ioctl =="
if dmesg 2>/dev/null | grep -q 'dxgk:.*Ioctl failed'; then
  bad "dmesg 에 dxgk ioctl 실패 있음:"
  dmesg 2>/dev/null | grep 'dxgk:.*Ioctl failed' | tail -3 | sed 's/^/         /'
elif ! dmesg 2>/dev/null | grep -q dxg; then
  info "dmesg 를 root 없이 못 읽음 — 'sudo dmesg | grep dxg' 로 직접 확인"
else
  ok "dxgk ioctl 실패 로그 없음"
fi

echo "== 3. Xwayland GPU 가속(glamor) =="
if grep -q 'falling back to sw' /mnt/wslg/stderr.log 2>/dev/null; then
  bad "Xwayland glamor 초기화 실패 → 모든 X 앱이 소프트웨어 렌더"
else
  ok "Xwayland glamor sw 폴백 로그 없음"
fi

echo "== 4. 실제 OpenGL 렌더러 =="
if command -v glxinfo >/dev/null 2>&1; then
  r=$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
  case "$r" in
    *llvmpipe*|*softpipe*|"") bad "renderer = ${r:-불명} (소프트웨어)";;
    *D3D12*|*NVIDIA*)         ok  "renderer = $r (GPU 가속!)";;
    *)                        info "renderer = $r";;
  esac
else
  info "glxinfo 없음 → 'sudo apt install mesa-utils'"
fi

echo "== 5. WSL 런타임 라이브러리 최신도 =="
d3d12_year=$(date -r /usr/lib/wsl/lib/libd3d12core.so +%Y 2>/dev/null || echo 0)
if [[ "$d3d12_year" -ge 2025 ]]; then ok "libd3d12core.so 빌드 연도 $d3d12_year"
else bad "libd3d12core.so 가 오래됨(${d3d12_year}) → wsl --update 로 갱신 필요"; fi

if command -v /usr/lib/wsl/lib/nvidia-smi >/dev/null 2>&1; then
  /usr/lib/wsl/lib/nvidia-smi 2>/dev/null | grep -E 'NVIDIA-SMI .* Driver Version' \
    | sed 's/^/  [ .. ] /'
fi

echo
echo "결과: OK $pass / FAIL $fail"
if [[ $fail -eq 0 ]]; then
  echo "GPU 렌더 경로가 살아 있습니다. 위 주석 5)번(qtwayland5 + gui_env 교체)을 진행하세요."
else
  echo "위 주석 1)~3) (wsl --update / NVIDIA 드라이버 클린 설치 / wsl --shutdown)을"
  echo "Windows 쪽에서 먼저 하세요. 그 전까지는 headless + RViz 로 사용하면 됩니다."
fi
