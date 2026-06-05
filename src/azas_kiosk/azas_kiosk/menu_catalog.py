from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class KioskMenuItem:
    recipe_id: str
    name: str
    color: str
    role: str
    description: str
    order_text: str


MENU_ITEMS: tuple[KioskMenuItem, ...] = (
    KioskMenuItem(
        recipe_id="recipe_01",
        name="레드 메뉴",
        color="red",
        role="주스",
        description="과일감이 선명하고 가볍게 마시기 좋은 메뉴",
        order_text="레드 메뉴 만들어줘",
    ),
    KioskMenuItem(
        recipe_id="recipe_02",
        name="옐로우 메뉴",
        color="yellow",
        role="시럽",
        description="달콤하고 부드러운 느낌이 강한 메뉴",
        order_text="옐로우 메뉴 만들어줘",
    ),
    KioskMenuItem(
        recipe_id="recipe_03",
        name="그린 메뉴",
        color="green",
        role="리큐르",
        description="향이 선명하고 깔끔한 여운이 있는 메뉴",
        order_text="그린 메뉴 만들어줘",
    ),
    KioskMenuItem(
        recipe_id="recipe_04",
        name="블루 메뉴",
        color="blue",
        role="럼",
        description="칵테일다운 존재감과 깊이가 있는 메뉴",
        order_text="블루 메뉴 만들어줘",
    ),
)


def build_menu_payload() -> list[dict[str, str]]:
    return [asdict(item) for item in MENU_ITEMS]
