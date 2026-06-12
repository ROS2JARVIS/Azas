from __future__ import annotations

from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - deterministic fallback below keeps tests importable.
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - source-tree tests do not need ROS sourced.
    get_package_share_directory = None


DISPENSER_ALIASES = {
    "red": ("1번", "일번", "디스펜서1", "디스펜서일", "빨강", "빨간색", "레드", "red", "빨간"),
    "yellow": ("2번", "이번", "디스펜서2", "디스펜서이", "노랑", "노란색", "옐로우", "yellow", "노란"),
    "green": ("3번", "삼번", "디스펜서3", "디스펜서삼", "초록", "초록색", "그린", "green", "녹색"),
    "blue": ("4번", "사번", "디스펜서4", "디스펜서사", "파랑", "파란색", "블루", "blue", "파란"),
}

# Backward-compatible import name. Parser output uses color strings because
# azas_dispenser launch parameters expect target_dispenser:=red|yellow|green|blue.
COLOR_ALIASES = DISPENSER_ALIASES

# Ingredient roles are symbolic voice/order semantics. Robot coordinates and
# calibration values are intentionally not stored here.
DISPENSER_ROLES = {
    "blue": {
        "role": "rum",
        "label": "럼",
        "levels": ("없음", "약하게", "보통", "강하게"),
    },
    "yellow": {
        "role": "syrup",
        "label": "시럽",
        "levels": ("적게", "보통", "많게"),
    },
    "green": {
        "role": "liqueur",
        "label": "리큐르",
        "levels": ("적게", "보통", "많게"),
    },
    "red": {
        "role": "juice",
        "label": "주스",
        "levels": ("적게", "보통", "많게"),
    },
}

DISPENSER_TRAITS = {
    "red": ("fruitiness", "freshness", "light", "sweetness"),
    "yellow": ("sweetness", "softness"),
    "green": ("aroma", "herbal", "bitterness"),
    "blue": ("alcohol", "depth", "bitterness"),
}

TRAIT_DISPLAY_NAMES = {
    "sweetness": "단맛",
    "fruitiness": "과일감",
    "freshness": "상큼함",
    "aroma": "향",
    "alcohol": "도수",
    "bitterness": "쓴맛",
    "softness": "부드러움",
    "light": "가벼움",
    "depth": "깊이감",
    "herbal": "허브향",
}

TRAIT_KEYWORDS = {
    "sweetness": (
        "달달",
        "달게",
        "달콤",
        "달아",
        "시럽",
        "기분안좋",
        "우울",
        "힘들",
        "피곤",
        "스트레스",
        "답답",
    ),
    "fruitiness": (
        "과일",
        "과일맛",
        "주스",
        "상큼",
        "새콤",
        "기분좋",
        "행복",
        "신나",
        "기뻐",
        "상쾌",
        "설레",
        "기분안좋",
        "우울",
        "힘들",
        "피곤",
    ),
    "freshness": ("상큼", "새콤", "산미", "산뜻", "상쾌"),
    "aroma": ("향", "향좋", "향진", "향강", "풍부", "리큐르", "기분좋", "행복", "신나"),
    "alcohol": (
        "도수쎈",
        "도수센",
        "도수쌘",
        "도수높",
        "쎈술",
        "센술",
        "쌘술",
        "쎈거",
        "센거",
        "쌘거",
        "더쎈",
        "더센",
        "더쌘",
        "강한술",
        "강한거",
        "더강한",
        "술강",
        "럼강",
        "강하게",
    ),
    "bitterness": ("쓴맛", "쓴술", "쌉싸름", "허브", "드라이"),
    "softness": ("부드럽", "순하게", "편한"),
    "light": ("가볍", "부담", "편한", "세지않", "안세"),
    "depth": ("깊", "묵직"),
    "herbal": ("허브", "리큐르", "향"),
}

AVOID_TRAIT_KEYWORDS = {
    "sweetness": ("덜달", "안달", "달지않", "시럽적"),
    "alcohol": (
        "무알콜",
        "논알콜",
        "알코올없이",
        "술없이",
        "럼없이",
        "술약",
        "럼약",
        "도수낮",
        "약하게",
        "세지않",
        "안세",
        "술은싫",
        "술싫",
        "독한술싫",
        "독한건싫",
    ),
    "bitterness": ("쓴맛싫", "쓴술싫", "쓴맛나는술은싫", "쓴맛나는술싫", "독한술싫", "독한건싫"),
    "aroma": ("향약", "리큐르적"),
}

RECIPE_ALIASES = {
    "recipe_01": ("1번", "일번", "레시피1", "recipe1", "recipe_01", "레드메뉴", "빨강메뉴", "빨간색메뉴"),
    "recipe_02": ("2번", "이번", "레시피2", "recipe2", "recipe_02", "옐로우메뉴", "노랑메뉴", "노란색메뉴"),
    "recipe_03": ("3번", "삼번", "레시피3", "recipe3", "recipe_03", "그린메뉴", "초록메뉴", "초록색메뉴"),
    "recipe_04": ("4번", "사번", "레시피4", "recipe4", "recipe_04", "블루메뉴", "파랑메뉴", "파란색메뉴"),
}

RECIPE_DISPLAY_NAMES = {
    "recipe_01": "레드 메뉴",
    "recipe_02": "옐로우 메뉴",
    "recipe_03": "그린 메뉴",
    "recipe_04": "블루 메뉴",
}

RECIPE_DESCRIPTIONS = {
    "recipe_01": "주스 중심이라 과일감이 선명하고 가볍게 마시기 좋습니다.",
    "recipe_02": "시럽 중심이라 달콤하고 부드러운 느낌이 강합니다.",
    "recipe_03": "리큐르 중심이라 향이 선명하고 깔끔한 여운이 있습니다.",
    "recipe_04": "럼 중심이라 칵테일다운 존재감과 깊이가 있습니다.",
}

RECIPE_DISPENSERS = {
    "recipe_01": ("red",),
    "recipe_02": ("yellow",),
    "recipe_03": ("green",),
    "recipe_04": ("blue",),
}
RECIPE_AMOUNTS: dict[str, dict[str, int]] = {}
RECIPE_METADATA: dict[str, dict[str, object]] = {}

MOOD_WORDS = (
    "기분",
    "우울",
    "슬퍼",
    "슬프",
    "힘들",
    "피곤",
    "지침",
    "행복",
    "신나",
    "기뻐",
    "상쾌",
    "답답",
    "스트레스",
    "설레",
)

RANDOM_RECIPE_WORDS = (
    "추천",
    "아무거나",
    "랜덤",
    "무작위",
    "골라",
    "골라줘",
    "알려줘",
)

REROLL_RECOMMENDATION_WORDS = (
    "다른거",
    "다른것",
    "다른메뉴",
    "다른걸",
    "다른걸로",
    "다른거로",
    "말고다른",
    "말고다른거",
    "말고다른메뉴",
    "새로추천",
    "다시추천",
)

PREFERENCE_WORDS = (
    "덜달",
    "안달",
    "달달",
    "달게",
    "달콤",
    "시럽",
    "상큼",
    "새콤",
    "신맛",
    "산미",
    "주스",
    "세지",
    "세지않",
    "안세",
    "부담",
    "가볍",
    "편한",
    "기분안좋",
    "안좋",
    "기분좋",
    "행복",
    "신나",
    "우울",
    "힘들",
    "피곤",
    "술약",
    "약하게",
    "술강",
    "강하게",
    "도수",
    "도수쎈",
    "도수센",
    "쎈거",
    "센거",
    "쌘거",
    "더쎈",
    "더센",
    "더쌘",
    "강한거",
    "럼",
    "무알콜",
    "논알콜",
    "알코올없이",
    "술없이",
    "리큐르",
    "쓴맛",
    "쓴술",
    "술은싫",
    "술싫",
    "독한술",
    "독한건싫",
    "과일",
    "과일맛",
    "향",
    "풍부",
    "진하게",
    "깔끔",
)

CONFIRM_WORDS = (
    "확인",
    "확인해",
    "확인해줘",
    "확정",
    "확정해",
    "확정해줘",
    "맞아",
    "맞아요",
    "맞습니다",
    "응",
    "응응",
    "네",
    "넵",
    "넹",
    "예",
    "예스",
    "yes",
    "ok",
    "okay",
    "오케이",
    "오키",
    "그래",
    "그렇게",
    "그렇게해",
    "그렇게해줘",
    "좋아",
    "좋아요",
    "좋습니다",
    "좋지",
    "괜찮아",
    "괜찮아요",
    "괜찮습니다",
    "알겠어",
    "알겠어요",
    "알겠습니다",
    "알았어",
    "알았어요",
    "알았습니다",
    "알겠",
    "알았",
    "오케",
    "콜",
    "가자",
    "시작",
    "시작해",
    "시작해줘",
    "진행",
    "진행해",
    "진행해줘",
    "계속해",
    "계속해줘",
    "계속",
)
CANCEL_WORDS = ("취소", "아니", "아니요", "멈춰", "중지", "그만", "정지")


def recipe_amounts(recipe_id: str | None) -> dict[str, int] | None:
    if not recipe_id:
        return None
    amounts = RECIPE_AMOUNTS.get(recipe_id)
    return dict(amounts) if amounts else None


def build_public_catalog() -> dict[str, object]:
    ingredients = {
        color: {
            "role": role.get("role", color),
            "label": role.get("label", color),
            "traits": list(DISPENSER_TRAITS.get(color, ())),
            "aliases": list(COLOR_ALIASES.get(color, ())),
        }
        for color, role in DISPENSER_ROLES.items()
    }
    recipes = []
    for recipe_id, dispenser_ids in RECIPE_DISPENSERS.items():
        metadata = RECIPE_METADATA.get(recipe_id, {})
        amounts = recipe_amounts(recipe_id) or {color: 1 for color in dispenser_ids}
        recipes.append(
            {
                "recipe_id": recipe_id,
                "name": RECIPE_DISPLAY_NAMES.get(recipe_id, recipe_id),
                "description": RECIPE_DESCRIPTIONS.get(recipe_id, ""),
                "aliases": list(RECIPE_ALIASES.get(recipe_id, ())),
                "dispenser_ids": list(dispenser_ids),
                "dispenser_amounts": amounts,
                "tags": list(metadata.get("tags", [])),
                "mood_tags": list(metadata.get("mood_tags", [])),
                "color": metadata.get("color", dispenser_ids[0] if dispenser_ids else ""),
                "sweetness": metadata.get("sweetness"),
                "acidity": metadata.get("acidity"),
                "strength": metadata.get("strength"),
            }
        )
    return {"ingredients": ingredients, "recipes": recipes}


def _candidate_config_paths() -> tuple[Path, ...]:
    paths = [Path(__file__).resolve().parents[1] / "config" / "recipes.yaml"]
    if get_package_share_directory is not None:
        try:
            paths.insert(0, Path(get_package_share_directory("azas_voice")) / "config" / "recipes.yaml")
        except Exception:
            pass
    return tuple(paths)


def _read_catalog_config() -> dict[str, Any] | None:
    if yaml is None:
        return None
    for path in _candidate_config_paths():
        if not path.is_file():
            continue
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if isinstance(payload, dict):
            return payload
    return None


def _dedupe(values: list[object] | tuple[object, ...]) -> tuple[str, ...]:
    result: list[str] = []
    for value in values:
        text = str(value).strip()
        if text and text not in result:
            result.append(text)
    return tuple(result)


def _recipe_number_aliases(recipe_id: str) -> tuple[str, ...]:
    digits = "".join(ch for ch in recipe_id if ch.isdigit())
    if not digits:
        return ()
    number = int(digits)
    korean_numbers = {
        1: "일",
        2: "이",
        3: "삼",
        4: "사",
        5: "오",
        6: "육",
        7: "칠",
        8: "팔",
        9: "구",
        10: "십",
        11: "십일",
        12: "십이",
        13: "십삼",
        14: "십사",
        15: "십오",
        16: "십육",
    }
    aliases = [f"{number}번", f"레시피{number}", f"recipe{number}", recipe_id]
    korean = korean_numbers.get(number)
    if korean:
        aliases.extend([f"{korean}번", f"{korean}번메뉴"])
    return tuple(aliases)


def _normalize_amounts(raw: object) -> dict[str, int]:
    if not isinstance(raw, dict):
        return {}
    amounts: dict[str, int] = {}
    for color in ("red", "yellow", "green", "blue"):
        try:
            amount = int(raw.get(color, 0))
        except (TypeError, ValueError):
            amount = 0
        amounts[color] = max(0, min(amount, 3))
    return amounts


def _apply_catalog_config() -> None:
    config = _read_catalog_config()
    if not config:
        return

    colors = config.get("colors")
    if isinstance(colors, dict):
        for color, block in colors.items():
            if color not in DISPENSER_ALIASES or not isinstance(block, dict):
                continue
            aliases = list(DISPENSER_ALIASES[color])
            aliases.extend(block.get("aliases", []) or [])
            aliases.extend([color, block.get("role", ""), block.get("ingredient_role", "")])
            DISPENSER_ALIASES[color] = _dedupe(tuple(aliases))
            DISPENSER_TRAITS[color] = _dedupe(tuple(block.get("traits", []) or DISPENSER_TRAITS.get(color, ())))
            role_name = str(block.get("role") or DISPENSER_ROLES[color].get("role") or color)
            label = str(block.get("ingredient_role") or DISPENSER_ROLES[color].get("label") or role_name)
            DISPENSER_ROLES[color] = {
                "role": role_name,
                "label": label,
                "levels": DISPENSER_ROLES[color].get("levels", ()),
            }

    recipes = config.get("recipes")
    if not isinstance(recipes, dict):
        return

    loaded_aliases: dict[str, tuple[str, ...]] = {}
    loaded_names: dict[str, str] = {}
    loaded_descriptions: dict[str, str] = {}
    loaded_dispensers: dict[str, tuple[str, ...]] = {}
    loaded_amounts: dict[str, dict[str, int]] = {}
    loaded_metadata: dict[str, dict[str, object]] = {}

    for recipe_id, block in recipes.items():
        if not isinstance(block, dict):
            continue
        recipe_key = str(recipe_id).strip()
        if not recipe_key:
            continue
        amounts = _normalize_amounts(block.get("dispenser_amounts"))
        raw_ids = block.get("dispenser_ids", [])
        dispenser_ids = [
            str(item).strip()
            for item in raw_ids
            if str(item).strip() in DISPENSER_ALIASES
        ]
        if amounts:
            dispenser_ids = [color for color in ("red", "yellow", "green", "blue") if amounts.get(color, 0) > 0]
        dispenser_ids = list(_dedupe(tuple(dispenser_ids)))
        if not dispenser_ids:
            continue

        name = str(block.get("name") or recipe_key)
        description = str(block.get("description") or "")
        aliases = list(block.get("aliases", []) or [])
        aliases.extend([recipe_key, name])
        aliases.extend(_recipe_number_aliases(recipe_key))

        loaded_aliases[recipe_key] = _dedupe(tuple(aliases))
        loaded_names[recipe_key] = name
        loaded_descriptions[recipe_key] = description
        loaded_dispensers[recipe_key] = tuple(dispenser_ids)
        if amounts:
            loaded_amounts[recipe_key] = amounts
        loaded_metadata[recipe_key] = {
            "tags": list(block.get("tags", []) or []),
            "mood_tags": list(block.get("mood_tags", []) or []),
            "color": block.get("color") or dispenser_ids[0],
            "sweetness": block.get("sweetness"),
            "acidity": block.get("acidity"),
            "strength": block.get("strength"),
        }

    if loaded_dispensers:
        RECIPE_ALIASES.clear()
        RECIPE_ALIASES.update(loaded_aliases)
        RECIPE_DISPLAY_NAMES.clear()
        RECIPE_DISPLAY_NAMES.update(loaded_names)
        RECIPE_DESCRIPTIONS.clear()
        RECIPE_DESCRIPTIONS.update(loaded_descriptions)
        RECIPE_DISPENSERS.clear()
        RECIPE_DISPENSERS.update(loaded_dispensers)
        RECIPE_AMOUNTS.clear()
        RECIPE_AMOUNTS.update(loaded_amounts)
        RECIPE_METADATA.clear()
        RECIPE_METADATA.update(loaded_metadata)


_apply_catalog_config()
