from pathlib import Path
import copy, hashlib, json, xml.etree.ElementTree as ET
root = Path(__file__).resolve().parent
repo = root.parents[3]
source = repo / "src/arena_gazebo/worlds/it_arena_official/world.sdf"
original = ET.parse(source)
variants = {}
for name in ("static_full", "physics_only", "no_physics", "empty_physics"):
    tree = copy.deepcopy(original)
    world = tree.getroot().find("world")
    for uri in world.findall(".//uri") + world.findall(".//albedo_map"):
        if uri.text and "://" not in uri.text and not Path(uri.text).is_absolute():
            uri.text = str((source.parent / uri.text).resolve())
    for plugin in list(world.findall("plugin")):
        is_physics = plugin.get("filename") == "gz-sim-physics-system"
        if (name in ("physics_only", "empty_physics") and not is_physics) or (name == "no_physics" and is_physics):
            world.remove(plugin)
    if name == "empty_physics":
        for model in list(world.findall("model")):
            world.remove(model)
    path = root / (name + ".sdf")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    variants[name] = {"models":len(world.findall("model")), "links":len(world.findall(".//link")),
                      "collisions":len(world.findall(".//collision"))}
(root / "variants.json").write_text(json.dumps({"source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"variants":variants},indent=2))
