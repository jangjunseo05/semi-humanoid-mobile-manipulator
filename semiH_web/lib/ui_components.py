"""공통 브랜딩 UI 컴포넌트 -- 로고+DAPIER 헤더, 스플래시 화면.

패턴 출처: `so101_web/dashboard/lib/ui_components.py`의 `render_brand_header()`/
`render_splash_screen()`을 실제로 읽고 그대로 재사용함
(`C:\\Users\\USER\\Desktop\\so101_web`). semiH_web은 현재 `pages/main.py`
하나뿐이지만, so101_web과 동일하게 페이지 파일이 아닌 별도 위치(`lib/`)에
둬서 나중에 페이지가 늘어나도 각 페이지가 이 두 함수만 불러 쓰면 되도록
했다(so101_web은 `dashboard/lib/`, semiH_web은 그 "dashboard/" 자리가 곧
프로젝트 루트라 `lib/`).

로고 파일은 `semiH_web/logo.png`(다색 꽃 로고 + "AI CAMPUS" 텍스트, 150x150,
배경 남색) -- 이미 존재하는 것을 확인하고 썼다. 배경색은 임의 추정하지
않고 PIL로 네 모서리 픽셀을 직접 실측했다(2026-08-18): (2,2)/(w-3,2)/
(2,h-3)/(w-3,h-3) 네 곳 전부 `#051644`로 동일 -- so101_web의
`_SPLASH_NAVY`와 같은 값이 나왔는데, 이건 같은 부트캠프 브랜드의 남색을
공유하는 것으로 보이며(추측), 이번 로고 파일에서 다시 직접 측정해 나온
값이라는 점이 중요하다(전 프로젝트 값을 그대로 베낀 게 아님).
"""

from __future__ import annotations

import base64
from pathlib import Path

import streamlit as st
from PIL import Image

_LOGO_PATH = Path(__file__).resolve().parents[1] / "logo.png"

# 로고 파일 코너 픽셀 실측값 -- 모듈 docstring 참고, 임의 추정 아님.
_SPLASH_NAVY = "#051644"
_GRID = "#e5e4e0"  # so101_web과 동일한 옅은 회색 구분선(브랜드 색과 무관, 순수 UI 톤)

TAGLINE = "SLAM, NAV, YOLO까지 한 번에"


@st.cache_data(show_spinner=False)
def _logo_base64() -> str | None:
    if not _LOGO_PATH.is_file():
        return None
    return base64.b64encode(_LOGO_PATH.read_bytes()).decode()


def get_logo_icon() -> Image.Image | str:
    """`st.set_page_config(page_icon=...)`용. 로고 파일이 없으면(예: 이
    저장소를 logo.png 없이 clone한 경우) 조용히 기본 이모지로 폴백 --
    브랜드 자산 누락이 앱 시작 자체를 막으면 안 되므로."""
    if _LOGO_PATH.is_file():
        return Image.open(_LOGO_PATH)
    return "🤖"


def render_brand_header() -> None:
    """페이지 최상단(제목보다 위)에서 호출 -- 로고 아이콘(36px) + "DAPIER"
    텍스트를 나란히 배치. 카드 role 색상 팔레트(action/settings/success/
    warning, `pages/main.py`의 `_ROLE_STYLES`)와 완전히 분리된 영역이다
    -- 로고 자체가 다색이라 배경색 블록으로 만들면 카드 색의 "의미 고정"
    원칙과 섞이므로, 배경 없이 로고+텍스트만 두고 얇은 회색 구분선으로
    아래 콘텐츠와 분리한다(so101_web과 동일한 이유).
    """
    logo_b64 = _logo_base64()
    logo_html = (
        f'<img src="data:image/png;base64,{logo_b64}" width="36" height="36" '
        f'style="border-radius:6px;flex-shrink:0;">'
        if logo_b64
        else ""
    )
    st.markdown(
        f'<div style="display:flex;align-items:center;gap:0.6em;margin-bottom:0.6em;">'
        f"{logo_html}"
        f'<span style="font-weight:700;font-size:1.2em;letter-spacing:0.04em;">DAPIER</span>'
        f"</div>"
        f'<hr style="margin:0 0 1.2em 0;border:none;border-top:1px solid {_GRID};">',
        unsafe_allow_html=True,
    )


def render_splash_screen() -> None:
    """최초 접속 시 전체 화면 스플래시. `position:fixed` 전체 화면
    덮개로 렌더링만 담당한다 -- 몇 초 뒤 사라지게 하는 sleep+rerun과
    "언제 다시 보여줄지"(세션당 1회 vs 매 새로고침) 판단은 호출부
    (`pages/main.py`)의 책임이다(so101_web과 동일한 역할 분리).

    세로 중앙 정렬 순서: 로고(72px) -> "DAPIER"(19px) -> 태그라인(36px,
    bold). 전부 so101_web `render_splash_screen()`과 동일한 크기 --
    요청받은 범위(로고 60~80px, DAPIER 18~20px, 태그라인 32~40px)에
    이미 들어맞아서 그대로 재사용했다.
    """
    logo_b64 = _logo_base64()
    logo_html = (
        f'<img src="data:image/png;base64,{logo_b64}" width="72" height="72" '
        f'style="border-radius:14px;">'
        if logo_b64
        else ""
    )
    st.markdown(
        f'<div style="position:fixed;top:0;left:0;width:100vw;height:100vh;'
        f'background-color:{_SPLASH_NAVY};z-index:999999;display:flex;'
        f'flex-direction:column;align-items:center;justify-content:center;">'
        f"{logo_html}"
        f'<div style="margin-top:0.9em;color:#ffffff;font-size:19px;'
        f'font-weight:600;letter-spacing:0.06em;">DAPIER</div>'
        f'<div style="margin-top:1.4em;color:#ffffff;font-size:36px;'
        f'font-weight:700;text-align:center;line-height:1.4;padding:0 1em;">'
        f"{TAGLINE}</div>"
        f"</div>",
        unsafe_allow_html=True,
    )
