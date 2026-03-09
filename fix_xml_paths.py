import xml.etree.ElementTree as ET
import re
import os

def fix_path(old_path):
    new_path = old_path
    
    # --- Collision (这些已经是 .stl 了，保持原样或修正路径) ---
    if "collision/body/" in old_path:
        new_path = old_path.replace("collision/body/", "body/v10/collision/")
        new_path = new_path.replace(".stl", "_symp.stl")
    elif "collision/arm/" in old_path:
        new_path = old_path.replace("collision/arm/", "arm/v10/collision/")
        new_path = new_path.replace(".stl", "_symp.stl")
    elif "collision/gripper/" in old_path:
        new_path = old_path.replace("collision/gripper/", "ee/openarm_hand/collision/")
        
    # --- Visual (关键修改：全部指向 .stl) ---
    elif "visual/body/" in old_path:
        # body/v10/visual/body_link0.stl
        new_path = "body/v10/visual/body_link0.stl"
        
    elif "visual/arm/" in old_path:
        match = re.search(r'link(\d+)', old_path)
        if match:
            link_num = match.group(1)
            new_path = f"arm/v10/visual/link{link_num}.stl"
            
    elif "visual/gripper/" in old_path:
        if "hand" in old_path:
            new_path = "ee/openarm_hand/visual/hand.stl"
        elif "finger" in old_path:
            new_path = "ee/openarm_hand/visual/finger.stl"
            
    return new_path

tree = ET.parse('model/openarm_bimanual.xml')
root = tree.getroot()

# 收集已经存在的 mesh name，防止重复处理
processed_meshes = set()

for mesh in root.findall(".//mesh"):
    old_file = mesh.get('file')
    if old_file:
        new_file = fix_path(old_file)
        if old_file != new_file:
            print(f"Fixing: {old_file} -> {new_file}")
            mesh.set('file', new_file)

# Save
output_path = 'model/openarm_bimanual_fixed.xml'
tree.write(output_path, encoding='utf-8', xml_declaration=True)
print(f"Saved {output_path}")