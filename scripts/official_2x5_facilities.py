"""v2026.11.01 공식 2×5 신호 파생본. 실제 코스에 없는 기둥은 명시적으로 제외하고 광원·마커는 보존합니다."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from experimental_facilities import add_grid_and_finish, profile_status

SIGNAL_STRUCTURE = ("tl_post", "tl_beam", "tl_panel")
LAMP_COLUMNS = (("lamp_red_1", "1 0 0 1"), ("lamp_red_2", "1 0 0 1"), ("lamp_red_3", "1 0 0 1"),
                ("lamp_red_4", "1 0 0 1"), ("lamp_green", "0 1 0 1"))
LAMP_ROWS = ("top", "bottom")
LAMP_OFF_RGBA = ".004 .004 .004 1"


def lamp_channels() -> list[dict]:
    """열 0–3은 빨강, 열 4는 초록. 이름은 공식 SDF 링크와 같습니다."""
    return [{"name": f"{prefix}_{row}", "column": column, "row": index, "lit_rgba": rgba}
            for column, (prefix, rgba) in enumerate(LAMP_COLUMNS) for index, row in enumerate(LAMP_ROWS)]


def configure_official_2x5(res: dict, profile: dict) -> dict:
    config = profile["facilities"]
    light = config["traffic_light"]
    if light.get("representation") != "official_2x5" or light.get("initial_state") != "off":
        raise ValueError("공식 2×5 신호 프로필은 소등 초기 상태만 지원합니다.")
    if "speed_bump" in config or "gantry_supports" in config or res["bumps"]:
        raise ValueError("공식 제거된 방지턱과 팀 임시 지지대를 v2026.11.01 파생본에 넣지 않습니다.")
    if type(light.get("remove_track_post", False)) is not bool:
        raise ValueError("remove_track_post는 bool이어야 합니다.")
    markers, grid = config["markers"], config["starting_grid"]
    if markers.get("mount_type") != "gantry" or markers.get("orientation_mode") != "official_pose" \
            or markers.get("representation") != "official_pbr_texture":
        raise ValueError("공식 마커의 설치 형식과 PBR 표현을 보존해야 합니다.")
    if grid.get("representation") != "official_filled_slots" or len(res["grid_slots"]) != 6:
        raise ValueError("공식 6칸 채움 그리드를 보존해야 합니다.")
    res["experimental_facilities"] = config
    return config


def _canonical(element) -> str:
    return ET.canonicalize(ET.tostring(element, encoding="unicode"), strip_text=True)


def verify_official_signal(source_world, runtime_world, *, remove_track_post=False) -> None:
    """명시한 기둥 제거 외에는 구조물·광원 pose/형상을 원본과 대조합니다."""
    query = "./world/model[@name='it_arena_track_static']"
    left, right = ET.parse(source_world).find(query), ET.parse(runtime_world).find(query)
    for name in SIGNAL_STRUCTURE:
        a, b = left.find(f"link[@name='{name}']"), right.find(f"link[@name='{name}']")
        if name == "tl_post" and remove_track_post:
            if b is not None:
                raise ValueError("제거 대상으로 등록한 tl_post가 실행 월드에 남았습니다.")
            continue
        if a is None or b is None or _canonical(a) != _canonical(b):
            raise ValueError(f"공식 신호 구조물 {name}이 변경되었습니다.")
    for channel in lamp_channels():
        name = channel["name"]
        a, b = left.find(f"link[@name='{name}']"), right.find(f"link[@name='{name}']")
        if a is None or b is None or a.findtext("pose") != b.findtext("pose"):
            raise ValueError(f"공식 광원 {name}의 위치가 변경되었습니다.")
        for visual_part in (a.find(f"visual[@name='{name}_vis']/geometry"), b.find(f"visual[@name='{name}_vis']/geometry")):
            if visual_part is None:
                raise ValueError(f"공식 광원 {name}의 시각 형상이 없습니다.")
        if _canonical(a.find(f"visual[@name='{name}_vis']/geometry")) != _canonical(b.find(f"visual[@name='{name}_vis']/geometry")):
            raise ValueError(f"공식 광원 {name}의 시각 형상이 변경되었습니다.")


def install_official_2x5(world_path, res: dict, generator) -> None:
    """사용자가 실제 없음을 확인한 기둥 제거·소등 재질·기존 결승 표시를 적용합니다."""
    tree = ET.parse(world_path)
    model = tree.find("./world/model[@name='it_arena_track_static']")
    if res["experimental_facilities"]["traffic_light"].get("remove_track_post", False):
        post = model.find("link[@name='tl_post']")
        if post is None:
            raise ValueError("공식 입력에서 예상한 tl_post를 찾지 못했습니다.")
        model.remove(post)  # visual과 collision을 함께 제거; 원본 ZIP은 읽기 전용
    for channel in lamp_channels():
        name = channel["name"]
        material = model.find(f"link[@name='{name}']/visual[@name='{name}_vis']/material")
        if material is None:
            raise ValueError(f"공식 광원 {name}의 재질이 없습니다.")
        if material.findtext("emissive") != channel["lit_rgba"]:
            raise ValueError(f"공식 광원 {name}의 색이 예상과 다릅니다.")
        material.find("ambient").text = material.find("diffuse").text = LAMP_OFF_RGBA
        material.find("emissive").text = "0 0 0 1"
    add_grid_and_finish(model, res, generator)
    ET.indent(tree, space="  ")
    tree.write(world_path, encoding="utf-8", xml_declaration=True)


def update_official_2x5_scene(scene: dict, res: dict) -> None:
    config = res["experimental_facilities"]
    scene["traffic_light"].pop("udp_port", None)
    scene["traffic_light"].update({
        "initial_state": "off", "state_control": "team_sim_material_service_official_links",
        "lamp_channels": [{key: item[key] for key in ("name", "column", "row")} for item in lamp_channels()],
        "udp_visual_controller_connected": False,
        "track_post_removed": config["traffic_light"].get("remove_track_post", False),
        "track_post_basis": ("user_confirmed_absent_2026_11_02"
                             if config["traffic_light"].get("remove_track_post", False) else "official_model"),
    })
    grid = config["starting_grid"]
    scene["starting_grid"]["paint"] = {
        "representation": "official_filled_slots", "source": "official world.sdf grid_slot_0..5 visuals",
        "length_m": grid["length_m"], "width_m": grid["width_m"],
        "paint_center_height_m": grid["center_height_m"], "paint_thickness_m": grid["thickness_m"],
        "material": "black", "collision": False, "numbering": "none",
    }
    scene["finish_line"] = res["finish_line"]
    cases = [{"name": f"signal_grid_{slot['index']}", "x": slot["x"], "y": slot["y"],
              "yaw_rad": slot["yaw_rad"], "expected_signal": "off"} for slot in scene["starting_grid"]["slots"]]
    scene["facility_inspection"] = {"status": profile_status(config), "config": config,
                                    "camera_cases": cases, "test_ground_truth_only": True}
