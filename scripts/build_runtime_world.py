#!/usr/bin/env python3
"""Build a Gazebo-friendly runtime world from the preserved organizer output.

The supplied SDF contains more than one thousand static links. Gazebo spends an
unreasonable amount of time creating a separate rigid-body entity for each one.
This script merges flat track / wall / grass / grid links into one static link,
while preserving each child geometry's model-relative pose. Semantic feature
links (traffic light and ArUco boards) remain separate for later control/tests.
"""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


MERGED_PREFIXES = (
    "ground_plane_",
    "surface_",
    "grass_",
    "walls_",
    "bump_",
    "grid_",
)


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    source_dir = (
        repo / "assets" / "track" / "source" / "it_arena_track" / "output_final"
    )
    destination_dir = (
        repo / "src" / "arena_gazebo" / "worlds" / "it_arena_track"
    )
    source_world = source_dir / "world.sdf"
    destination_world = destination_dir / "world.sdf"

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source_world, parser=parser)
    root = tree.getroot()
    world = root.find("./world")
    if world is None:
        raise RuntimeError("world element not found in supplied SDF")
    model = root.find("./world/model[@name='it_arena_track_static']")
    if model is None:
        raise RuntimeError("static track model not found in supplied world")

    merged_link = ET.Element("link", {"name": "track_geometry_merged"})
    links = list(model.findall("link"))
    merged_count = 0

    for link in links:
        link_name = link.attrib.get("name", "unnamed")
        if not link_name.startswith(MERGED_PREFIXES):
            continue

        pose = link.findtext("pose", default="0 0 0 0 0 0").strip()
        for element_name in ("collision", "visual"):
            for child in list(link.findall(element_name)):
                old_name = child.attrib.get("name", element_name)
                child.set("name", f"{link_name}__{old_name}")
                child_pose = ET.Element("pose")
                child_pose.text = pose
                child.insert(0, child_pose)
                link.remove(child)
                merged_link.append(child)

        model.remove(link)
        merged_count += 1

    static_element = model.find("static")
    insert_at = list(model).index(static_element) + 1 if static_element is not None else 0
    model.insert(insert_at, merged_link)

    for albedo_map in model.findall(".//albedo_map"):
        if albedo_map.text:
            albedo_map.text = albedo_map.text.replace("../aruco/", "aruco/")

    # The supplied legacy script materials have a URI but no required name, so
    # strict SDFormat rejects the whole world. PBR albedo maps remain in place.
    for material in model.findall(".//material"):
        for script in list(material.findall("script")):
            material.remove(script)

    # Defining any world plugin disables Gazebo's default server plugin list,
    # so keep the normal baseline and add Sensors explicitly for D435i output.
    plugin_specs = [
        ("gz-sim-physics-system", "gz::sim::systems::Physics", None),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", None),
        (
            "gz-sim-scene-broadcaster-system",
            "gz::sim::systems::SceneBroadcaster",
            None,
        ),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors", "ogre2"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu", None),
    ]
    insert_at = list(world).index(world.find("physics")) + 1
    for filename, name, render_engine in plugin_specs:
        plugin = ET.Element("plugin", {"filename": filename, "name": name})
        if render_engine is not None:
            engine = ET.SubElement(plugin, "render_engine")
            engine.text = render_engine
        world.insert(insert_at, plugin)
        insert_at += 1

    destination_dir.mkdir(parents=True, exist_ok=True)
    texture_destination = destination_dir / "aruco"
    texture_destination.mkdir(parents=True, exist_ok=True)
    for texture in sorted((source_dir / "aruco").glob("aruco_id*.png")):
        shutil.copy2(texture, texture_destination / texture.name)

    ET.indent(tree, space="  ")
    tree.write(destination_world, encoding="utf-8", xml_declaration=True)

    remaining_links = len(model.findall("link"))
    collision_count = len(model.findall(".//collision"))
    visual_count = len(model.findall(".//visual"))
    print(
        f"Runtime world generated: merged {merged_count} links; "
        f"{remaining_links} links, {collision_count} collisions, "
        f"{visual_count} visuals remain."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
