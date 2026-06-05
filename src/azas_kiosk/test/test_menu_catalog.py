from azas_kiosk.menu_catalog import build_menu_payload


def test_menu_payload_has_four_symbolic_recipes():
    menus = build_menu_payload()

    assert [menu["recipe_id"] for menu in menus] == [
        "recipe_01",
        "recipe_02",
        "recipe_03",
        "recipe_04",
    ]
    assert all("order_text" in menu for menu in menus)
