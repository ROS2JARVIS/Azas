DISPENSER_ALIASES = {
    "1": ("1번", "일번", "디스펜서1", "디스펜서일", "노랑", "노란색", "옐로우", "yellow", "노란"),
    "2": ("2번", "이번", "디스펜서2", "디스펜서이", "빨강", "빨간색", "레드", "red", "빨간"),
    "3": ("3번", "삼번", "디스펜서3", "디스펜서삼", "파랑", "파란색", "블루", "blue", "파란"),
    "4": ("4번", "사번", "디스펜서4", "디스펜서사", "초록", "초록색", "그린", "green", "녹색"),
}

# Backward-compatible import name. Parser output still uses fixed dispenser
# numbers only; color words are accepted only as aliases for those numbers.
COLOR_ALIASES = DISPENSER_ALIASES

# Recipe names and actual ingredients are intentionally symbolic until the team
# confirms which ingredient is loaded into each color-sticker dispenser.
RECIPE_ALIASES = {
    "recipe_01": ("1번", "일번", "레시피1", "recipe1", "recipe_01"),
    "recipe_02": ("2번", "이번", "레시피2", "recipe2", "recipe_02"),
    "recipe_03": ("3번", "삼번", "레시피3", "recipe3", "recipe_03"),
    "recipe_04": ("4번", "사번", "레시피4", "recipe4", "recipe_04"),
    "recipe_05": ("5번", "오번", "레시피5", "recipe5", "recipe_05"),
    "recipe_06": ("6번", "육번", "레시피6", "recipe6", "recipe_06"),
    "recipe_07": ("7번", "칠번", "레시피7", "recipe7", "recipe_07"),
    "recipe_08": ("8번", "팔번", "레시피8", "recipe8", "recipe_08"),
    "recipe_09": ("9번", "구번", "레시피9", "recipe9", "recipe_09"),
    "recipe_10": ("10번", "십번", "레시피10", "recipe10", "recipe_10"),
    "recipe_11": ("11번", "십일번", "레시피11", "recipe11", "recipe_11"),
    "recipe_12": ("12번", "십이번", "레시피12", "recipe12", "recipe_12"),
    "recipe_13": ("13번", "십삼번", "레시피13", "recipe13", "recipe_13"),
    "recipe_14": ("14번", "십사번", "레시피14", "recipe14", "recipe_14"),
    "recipe_15": ("15번", "십오번", "레시피15", "recipe15", "recipe_15"),
    "recipe_16": ("16번", "십육번", "레시피16", "recipe16", "recipe_16"),
}

RECIPE_DISPLAY_NAMES = {
    "recipe_01": "선셋 믹스",
    "recipe_02": "블루 라군",
    "recipe_03": "그린 스파클",
    "recipe_04": "레드 펀치",
    "recipe_05": "옐로우 브리즈",
    "recipe_06": "시트러스 쿨러",
    "recipe_07": "베리 블루",
    "recipe_08": "민트 선라이즈",
    "recipe_09": "트로피컬 무드",
    "recipe_10": "라임 레드",
    "recipe_11": "오션 옐로우",
    "recipe_12": "포레스트 펀치",
    "recipe_13": "스윗 밸런스",
    "recipe_14": "프레시 믹스",
    "recipe_15": "파티 컬러",
    "recipe_16": "랜덤 시그니처",
}

RECIPE_DISPENSERS = {
    "recipe_01": ("1", "2"),
    "recipe_02": ("3", "4"),
    "recipe_03": ("4", "1"),
    "recipe_04": ("2", "3"),
    "recipe_05": ("1", "4"),
    "recipe_06": ("1", "3"),
    "recipe_07": ("2", "3", "4"),
    "recipe_08": ("4", "1", "2"),
    "recipe_09": ("1", "2", "3"),
    "recipe_10": ("2", "4"),
    "recipe_11": ("3", "1", "4"),
    "recipe_12": ("4", "2"),
    "recipe_13": ("1", "3", "2"),
    "recipe_14": ("4", "3", "1"),
    "recipe_15": ("2", "1", "4", "3"),
    "recipe_16": ("3", "2", "1"),
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

CONFIRM_WORDS = ("확인", "맞아", "맞습니다", "응", "네", "예", "좋아", "시작")
CANCEL_WORDS = ("취소", "아니", "아니요", "멈춰", "중지", "그만", "정지")
