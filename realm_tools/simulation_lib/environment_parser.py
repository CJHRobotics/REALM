import xml.etree.ElementTree as ET


def parse_wall(xml_wall):
    return {
        'x1':        float(xml_wall.get('x1')),
        'y1':        float(xml_wall.get('y1')),
        'x2':        float(xml_wall.get('x2')),
        'y2':        float(xml_wall.get('y2')),
        'height':    float(xml_wall.get('height', 0.3)),
        'width':     float(xml_wall.get('width', 0.012)),
        'wall_type': xml_wall.get('type', 'obstacle'),
        'texture':   xml_wall.get('texture', None),
    }


def parse_all_walls(xml_root):
    return [parse_wall(w) for w in xml_root.findall('wall')]


def parse_landmark(xml_landmark):
    landmark_type = xml_landmark.get('type')
    data = {
        'type':   landmark_type,
        'x':      float(xml_landmark.get('x')),
        'y':      float(xml_landmark.get('y')),
        'theta':  float(xml_landmark.get('theta', 0.0)),
        'height': float(xml_landmark.get('height', 1.0)),
        'red':    float(xml_landmark.get('red', 1.0)),
        'green':  float(xml_landmark.get('green', 1.0)),
        'blue':   float(xml_landmark.get('blue', 1.0)),
    }
    if landmark_type == 'cylinder':
        data['radius'] = float(xml_landmark.get('radius', 0.25))
    elif landmark_type == 'panel':
        data['width']   = float(xml_landmark.get('width', 1.0))
        data['texture'] = xml_landmark.get('texture', '')
    return data


def parse_all_landmarks(xml_root):
    return [parse_landmark(lm) for lm in xml_root.findall('landmark')]


def parse_position(xml_pos):
    return {
        'x':     float(xml_pos.get('x')),
        'y':     float(xml_pos.get('y')),
        'theta': float(xml_pos.get('theta')),
    }


def parse_all_positions(xml_positions):
    if xml_positions is None:
        return []
    return [parse_position(p) for p in xml_positions.findall('pos')]


def parse_goal(xml_goal):
    return {
        'id': int(xml_goal.get('id', 0)),
        'x':  float(xml_goal.get('x')),
        'y':  float(xml_goal.get('y')),
    }


def parse_all_goals(xml_root):
    return [parse_goal(g) for g in xml_root.findall('goal')]


def parse_environment(file):
    root = ET.parse(file).getroot()
    return (
        parse_all_walls(root),
        parse_all_landmarks(root),
        parse_all_goals(root),
        parse_all_positions(root.find('train_start_positions')),
        parse_all_positions(root.find('test_start_positions')),
    )
