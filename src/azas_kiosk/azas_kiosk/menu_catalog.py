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
        name="베리 선셋",
        color="red",
        role="달콤한 과일감",
        description="붉은 베리 톤의 산뜻하고 가벼운 시그니처 칵테일",
        order_text="레드 메뉴 만들어줘",
    ),
    KioskMenuItem(
        recipe_id="recipe_02",
        name="시트러스 글로우",
        color="yellow",
        role="밝은 달콤함",
        description="시트러스처럼 밝고 부드럽게 마무리되는 칵테일",
        order_text="옐로우 메뉴 만들어줘",
    ),
    KioskMenuItem(
        recipe_id="recipe_03",
        name="허브 가든",
        color="green",
        role="허브 아로마",
        description="은은한 향과 깔끔한 여운을 살린 그린 칵테일",
        order_text="그린 메뉴 만들어줘",
    ),
    KioskMenuItem(
        recipe_id="recipe_04",
        name="오션 브리즈",
        color="blue",
        role="시원한 깊이감",
        description="차분한 블루 톤에 깊이감을 더한 시원한 칵테일",
        order_text="블루 메뉴 만들어줘",
    ),
)


def build_menu_payload() -> list[dict[str, str]]:
    return [asdict(item) for item in MENU_ITEMS]
