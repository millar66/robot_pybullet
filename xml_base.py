#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 19:22:27 2023

@author: lihui.liu
"""

import xml.etree.ElementTree as ET
from pprint import pprint
import os
import xml.dom.minidom
import shutil

shutil.copyfile("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/000PSM_10.SLDASM.urdf",
                "/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.urdf")

if os.path.exists("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.urdf"):
    os.replace("/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.urdf",
               "/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.xml")

tree = ET.parse('/home/lihui.liu/anaconda3/envs/anaconda_robot/lib/python3.9/site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/test.xml')

# 解析XML字符串并返回根元素的Element对象
# xml_string = '<root><element>Value</element></root>'
# root = ET.fromstring(xml_string)

# 获取XML文档的根元素
root = tree.getroot()

# 查找具有指定标签的第一个子元素
# element = root.find('link')
elements = root.findall('joint')
# origin_robot = joint_origins[0][0].get('xyz')
# joint_origins = element.find('origin')
joint_origins_list = [elements[i].findall('origin') for i in range(len(elements))]
joint_origins = [0] * len(joint_origins_list)

joint_limit = [0] * 11
joint_limit = [elements[i].findall('limit') for i in range(11)]

for i in range(len(joint_origins_list)):
    joint_origins[i] = joint_origins_list[i][0].get('xyz')
pprint(joint_origins)

# 可以使用元素对象的.text属性修改元素的文本内容，使用.set()方法修改元素的属性。
joint_origins_list[3][0].set('xyz', '10000 2000 308880')

tree.write('./site-packages/pybullet_data/aaa/000PSM_10.SLDASM/urdf/modified.urdf')

mass = xmlDoc.getElementsByTagName("mass")[robot_id].getAttribute("value")

# 查找具有指定标签的所有子元素
# elements = root.findall('joint')

# 获取元素的指定属性值
# attribute_value = elements.get('xyz')

# 可以使用元素对象的`.text`属性访问元素的文本内容，使用`.attrib`属性访问元素的属性。
# element = root.find('joint')
# if element is not None:
#     text = element.text
#     attributes = element.attrib
    
# for child in root:
#     print('Tag:', child.tag)
#     print('Text:', child.text)
#     print('Attributes:', child.attrib)
    
# 写入新的元素
# 可以创建新的元素对象，使用Element()函数或直接构造Element对象，并设置其标签、文本和属性。然后使用根元素的.append()方法将新元素添加为子元素。
# new_element = ET.Element('new_element')
# new_element.text = 'New element text'
# new_element.set('attribute_name', 'attribute_value')
# root.append(new_element)

# 使用根元素的.remove()方法删除指定的子元素。

# child_to_remove = root.find('element_to_remove')
# if child_to_remove is not None:
#     root.remove(child_to_remove)



    
    
    
    
    
    
    
    
    
    