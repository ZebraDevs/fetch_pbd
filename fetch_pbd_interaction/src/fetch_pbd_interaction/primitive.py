#!/usr/bin/env python
'''Abstract class defining an interface to primitives
'''

from abc import ABCMeta, abstractmethod

class Primitive:
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def build_from_json(self, json):
        pass

    @abstractmethod
    def get_json(self):
        pass

    @abstractmethod
    def check_pre_condition(self):
        pass

    @abstractmethod
    def check_post_condition(self):
        pass

    @abstractmethod
    def add_marker_callbacks(self, click_cb,
                    delete_cb,
                    pose_change_cb,
                    action_change_cb):
        pass

    @abstractmethod
    def show_marker(self):
        pass

    @abstractmethod
    def hide_marker(self):
        pass

    @abstractmethod
    def marker_visible(self):
        pass

    @abstractmethod
    def update_ref_frames(self):
        pass

    @abstractmethod
    def change_ref_frame(self, ref_type, landmark):
        pass

    @abstractmethod
    def select(self, is_selected):
        pass

    @abstractmethod
    def is_selected(self):
        pass

    @abstractmethod
    def is_control_visible(self):
        pass

    @abstractmethod
    def set_control_visible(self, visible=True):
        pass

    @abstractmethod
    def update_viz(self, check_reachable=True):
        pass

    @abstractmethod
    def get_primitive_number(self):
        pass

    @abstractmethod
    def set_primitive_number(self, number):
        pass

    @abstractmethod
    def is_object_required(self):
        pass

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def head_busy(self):
        pass

    @abstractmethod
    def is_reachable(self):
        pass

    @abstractmethod
    def get_relative_pose(self, use_final=True):
        pass

    @abstractmethod
    def get_absolute_pose(self):
        pass

    @abstractmethod
    def get_absolute_marker_pose(self, use_final=True):
        pass

    @abstractmethod
    def get_absolute_marker_position(self, use_final=True):
        pass

    @abstractmethod
    def decrease_id(self):
        pass

    @abstractmethod
    def set_name(self, name):
        pass

    @abstractmethod
    def get_name(self):
        pass

    @abstractmethod
    def get_number(self):
        pass

    @abstractmethod
    def set_pose(self, pose):
        pass

    @abstractmethod
    def pose_editable(self):
        pass

    @abstractmethod
    def get_ref_type(self):
        pass



