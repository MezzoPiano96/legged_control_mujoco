#!/usr/bin/env python3
import xml.etree.ElementTree as ET

# 加载原始模型
tree = ET.parse('a1_backup.xml')
root = tree.getroot()

# 1. 在 trunk body 中添加 IMU site
trunk = root.find(".//body[@name='trunk']")
if trunk is not None:
    imu_site = ET.SubElement(trunk, 'site', {
        'name': 'imu_site',
        'pos': '0 0 0',
        'size': '0.01'
    })
    print("✅ Added IMU site to trunk")

# 2. 为每个足端 geom 添加名称
feet = ['FR', 'FL', 'RR', 'RL']
for foot in feet:
    calf_body = root.find(f".//body[@name='{foot}_calf']")
    if calf_body is not None:
        for geom in calf_body.findall('geom'):
            if geom.get('class') == 'foot':
                geom.set('name', f'{foot}_FOOT')
                print(f"✅ Named {foot}_FOOT geom")

# 3. 添加 sensor 标签（在 </worldbody> 之后）
worldbody = root.find('worldbody')
if worldbody is not None:
    # 找到 worldbody 在 root 中的位置
    wb_index = list(root).index(worldbody)
    
    # 创建 sensor 元素
    sensor = ET.Element('sensor')
    
    # 添加 IMU 传感器
    ET.SubElement(sensor, 'framequat', {
        'name': 'base_orientation',
        'objtype': 'site',
        'objname': 'imu_site'
    })
    ET.SubElement(sensor, 'gyro', {
        'name': 'base_gyro',
        'site': 'imu_site'
    })
    ET.SubElement(sensor, 'accelerometer', {
        'name': 'base_accel',
        'site': 'imu_site'
    })
    
    # 插入到 worldbody 之后
    root.insert(wb_index + 1, sensor)
    print("✅ Added sensor definitions")

# 保存修改后的模型
tree.write('a1.xml', encoding='unicode', xml_declaration=True)
print("\n✅ Model modified successfully!")
print("📝 Changes:")
print("   - Added IMU site to trunk body")
print("   - Named foot geoms: FR_FOOT, FL_FOOT, RR_FOOT, RL_FOOT")
print("   - Added IMU sensors: framequat, gyro, accelerometer")
