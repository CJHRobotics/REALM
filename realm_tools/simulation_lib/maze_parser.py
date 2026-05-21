import os
import xml.etree.ElementTree as ET

import pandas as pd


def parse_goal(xml_goal):
    fid = int(xml_goal.get('id'))
    x = float(xml_goal.get('x'))
    y = float(xml_goal.get('y'))
    return pd.DataFrame(data=[[fid, x, y]], columns=['id', 'x', 'y'])


def parse_all_goals(root):
    return pd.concat([pd.DataFrame(columns=['id', 'x', 'y'])] + [parse_goal(xml_goal) for xml_goal in
                                                                 root.findall('goal')]).reset_index(drop=True)


def parse_all_goals(root):
    # Define the columns for the DataFrame
    columns = ['id', 'x', 'y']

    # Parse each goal and create a list of DataFrames
    data_frames = [parse_goal(xml_goal) for xml_goal in root.findall('goal')]

    # Check if there are any valid data frames to concatenate
    valid_data_frames = [df for df in data_frames if not df.empty and not df.isna().all().all()]
    if not valid_data_frames:
        return pd.DataFrame(columns=columns)

    # Concatenate the valid data frames
    return pd.concat(valid_data_frames, ignore_index=True)


def parse_wall(xml_wall):
    x1 = float(xml_wall.get('x1'))
    y1 = float(xml_wall.get('y1'))
    x2 = float(xml_wall.get('x2'))
    y2 = float(xml_wall.get('y2'))
    width = float(xml_wall.get('width', 0.012))  # Default width is 0.012 if not specified
    data = [[x1, y1, x2, y2, width]]
    return pd.DataFrame(data=data, columns=['x1', 'y1', 'x2', 'y2', 'width'])


def parse_all_obsticles(xml_root):
    # Define the columns for the DataFrame
    columns = ['x1', 'y1', 'x2', 'y2', 'width']

    # Parse each wall and create a list of DataFrames
    data_frames = [parse_wall(xml_wall) for xml_wall in xml_root.findall('wall')]

    # Check if there are any valid data frames to concatenate
    valid_data_frames = [df for df in data_frames if not df.empty and not df.isna().all().all()]
    if not valid_data_frames:
        return pd.DataFrame(columns=columns)

    # Concatenate the valid data frames
    return pd.concat(valid_data_frames, ignore_index=True)


def parse_cylinder_landmark(xml_cylinder_landmark):
    data = [[float(xml_cylinder_landmark.get(data)) for data in ['x', 'y', 'height', 'red', 'green', 'blue']]]
    return pd.DataFrame(data=data, columns=['x', 'y', 'height', 'red', 'green', 'blue'])

def parse_all_cylinder_landmarks(xml_root):
    # Create a list of DataFrames for each landmark
    cylinder_landmark_dataframes = [parse_cylinder_landmark(xml_landmark) for xml_landmark in xml_root.findall('cylinder_landmark')]

    # Filter out any empty or None DataFrames from parse_landmark results
    cylinder_landmark_dataframes = [df for df in cylinder_landmark_dataframes if not df.empty]

    # If no landmarks are found, return an empty DataFrame with the specified columns
    if not cylinder_landmark_dataframes:
        return pd.DataFrame(columns=['x', 'y', 'height', 'red', 'green', 'blue'])

    # Concatenate the landmark DataFrames and reset the index
    return pd.concat(cylinder_landmark_dataframes, ignore_index=True)

def parse_tag_landmark(xml_cylinder_landmark):
    data = [[float(xml_cylinder_landmark.get(data)) for data in ['x', 'y', 'theta', 'tag_id', 'height', 'width', 'red', 'green', 'blue']]]
    return pd.DataFrame(data=data, columns=['x', 'y', 'theta', 'tag_id', 'height', 'width', 'red', 'green', 'blue'])

def parse_all_tag_landmarks(xml_root):
    # Create a list of DataFrames for each landmark
    tag_landmark_dataframes = [parse_tag_landmark(xml_tag_landmark) for xml_tag_landmark in xml_root.findall('tag_landmark')]

    # Filter out any empty or None DataFrames from parse_landmark results
    tag_landmark_dataframes = [df for df in tag_landmark_dataframes if not df.empty]

    # If no landmarks are found, return an empty DataFrame with the specified columns
    if not tag_landmark_dataframes:
        return pd.DataFrame(columns=['x', 'y', 'theta', 'tag_id', 'height', 'width', 'red', 'green', 'blue'])

    # Concatenate the landmark DataFrames and reset the index
    return pd.concat(tag_landmark_dataframes, ignore_index=True)
def parse_position(xml_position):
    return pd.DataFrame(data=[[float(xml_position.get(p)) for p in ['x', 'y', 'theta']]], columns=['x', 'y', 'theta'])


def parse_all_positions(xml_positions):
    # Define the columns for the DataFrame
    columns = ['x', 'y', 'theta']

    # Handle the case where xml_positions is None
    if xml_positions is None or not xml_positions.findall('pos'):
        return pd.DataFrame(columns=columns)

    # Parse each position and create a list of DataFrames
    data_frames = [parse_position(xml_pos) for xml_pos in xml_positions.findall('pos')]

    # Check if there are any valid data frames to concatenate
    valid_data_frames = [df for df in data_frames if not df.empty and not df.isna().all().all()]
    if not valid_data_frames:
        return pd.DataFrame(columns=columns)

    # Concatenate the valid data frames
    return pd.concat(valid_data_frames, ignore_index=True)


def parse_maze(file):
    root = ET.parse(file).getroot()
    experiment_start_positions = parse_all_positions(root.find('experimentStartPositions'))
    habituation_start_positions = parse_all_positions(root.find('habituationStartPositions'))
    walls = parse_all_obsticles(root)
    goals = parse_all_goals(root)
    cylinder_landmarks = parse_all_cylinder_landmarks(root)
    tag_landmarks = parse_all_tag_landmarks(root)

    return walls, goals, experiment_start_positions, habituation_start_positions, cylinder_landmarks, tag_landmarks


def parse_maze_for_wavefront(file):
    root = ET.parse(file).getroot()
    experiment_start_positions = parse_all_positions(root.find('experimentStartPositions'))
    habituation_start_positions = parse_all_positions(root.find('habituationStartPositions'))
    starting_positions = pd.concat([experiment_start_positions, habituation_start_positions])
    walls = parse_all_obsticles(root)
    goals = parse_all_goals(root)
    cylinder_landmarks = parse_all_cylinder_landmarks(root)
    tag_landmarks = parse_all_tag_landmarks(root)

    return walls, goals, starting_positions
