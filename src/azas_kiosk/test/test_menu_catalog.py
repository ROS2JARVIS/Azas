from azas_kiosk.menu_catalog import build_menu_payload
from azas_kiosk.kiosk_node import _command_from_request


def test_menu_payload_has_four_symbolic_recipes():
    menus = build_menu_payload()

    assert [menu["recipe_id"] for menu in menus] == [
        "recipe_01",
        "recipe_02",
        "recipe_03",
        "recipe_04",
    ]
    assert [menu["name"] for menu in menus] == [
        "베리 선셋",
        "시트러스 글로우",
        "허브 가든",
        "오션 브리즈",
    ]
    assert all("order_text" in menu for menu in menus)


def test_order_request_sets_local_kiosk_feedback():
    result = _command_from_request("/api/order", {"recipe_id": "recipe_01"})

    assert result["text"] == "레드 메뉴 만들어줘"
    assert result["selected_recipe_id"] == "recipe_01"
    assert "베리 선셋" in result["local_confirmation"]


def test_cancel_request_clears_selected_menu():
    result = _command_from_request("/api/cancel", {})

    assert result["text"] == "취소"
    assert result["clear_selection"]
