DISPENSER_ALIASES = {
    "red": ("1번", "일번", "디스펜서1", "디스펜서일", "빨강", "빨간색", "레드", "red", "빨간"),
    "yellow": ("2번", "이번", "디스펜서2", "디스펜서이", "노랑", "노란색", "옐로우", "yellow", "노란"),
    "green": ("3번", "삼번", "디스펜서3", "디스펜서삼", "초록", "초록색", "그린", "green", "녹색"),
    "blue": ("4번", "사번", "디스펜서4", "디스펜서사", "파랑", "파란색", "블루", "blue", "파란"),
}

# Backward-compatible import name. Parser output uses color strings because
# azas_dispenser launch parameters expect target_dispenser:=red|yellow|green|blue.
COLOR_ALIASES = DISPENSER_ALIASES

# Recipe names and actual ingredients are intentionally symbolic until the team
# confirms which ingredient is loaded into each color-sticker dispenser.
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

RECIPE_DISPENSERS = {
    "recipe_01": ("red",),
    "recipe_02": ("yellow",),
    "recipe_03": ("green",),
    "recipe_04": ("blue",),
}

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

CONFIRM_WORDS = (
    "확인",
    "맞아",
    "맞습니다",
    "응",
    "네",
    "예",
    "좋아",
    "시작",
    "진행",
    "진행해",
    "진행해줘",
    "계속",
)
CANCEL_WORDS = ("취소", "아니", "아니요", "멈춰", "중지", "그만", "정지")
