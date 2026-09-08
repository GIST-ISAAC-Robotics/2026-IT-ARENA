from pathlib import Path
import xml.etree.ElementTree as ET
root = Path(__file__).resolve().parent
for detector in ("ode", "bullet", "fcl", "dart"):
    tree = ET.parse(root / "physics_only.sdf")
    physics = tree.getroot().find("world/physics")
    dart = ET.SubElement(physics, "dart")
    ET.SubElement(dart, "collision_detector").text = detector
    tree.write(root / f"detector_{detector}.sdf", encoding="utf-8", xml_declaration=True)
