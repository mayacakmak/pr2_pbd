import os.path
from os import listdir
from os.path import isfile, join
from xml.etree.ElementTree import ElementTree, TreeBuilder

class Action:
    ACTION_DIRECTORY = "/home/vladimir/pbd_actions/"
    FILE_EXTENSION = ".xml"

    ACTION_QUEUE = 0
    POSE = 1
    GRIPPER = 2
    TRAJECTORY = 3
    
    def get_file(self, action):
        return Action.ACTION_DIRECTORY + str(action) + ".xml"
    
    def __init__(self, id=None):
        self.type = 0
        self.name = "unnamed"
        if (id == None):
            '''Find next available id from files'''
            id = 0
            while (os.path.isfile(self.get_file(id))):
                id += 1
        else:
            if (os.path.isfile(self.get_file(id))):
                tree = ElementTree()
                tree.parse(self.get_file(id))
                root = tree.getroot()
                self.type = int(root.get("type"))
                self.name = root.find("name").text
                self._read_type(root)
        
        self.id = id
    
    def _read_type(self, root):
        props = { "position" : ["x", "y", "z"], "orientation": ["x", "y", "z", "w" ] }
        def get_pose(el):
            pose = {}
            for prop in props:
                self.pose[prop] = {}
                for prop2 in props[prop]:
                    self.pose[prop][prop2] = (
                        float(el.find(prop).find(prop2).text))
            return pose
        if (self.type == Action.ACTION_QUEUE):
            self.actions = map(lambda el: int(el.text), list(root.find("actions")))
        elif (self.type == Action.POSE):
            self.pose = get_pose(root.find("pose"))
        elif (self.type == Action.GRIPPER):
            self.is_open = bool(root.find("gripper").find("is_open").text)
            self.arm_index = int(root.find("gripper").find("arm_index").text)
        elif (self.type == Action.TRAJECTORY):
            self.poses = map(lambda pose_el: get_pose, list(root.find("poses")))
    
    def _write_type(self, builder):
        props = { "position" : ["x", "y", "z"], "orientation": ["x", "y", "z", "w" ] }
        def write_pose(pose):
            for prop in props:
                builder.start(prop, {})
                for prop2 in props[prop]:
                    builder.start(prop2, {})
                    builder.data(str(pose[prop][prop2]))
                    builder.end(prop2)
                builder.end(prop)
        if (self.type == Action.ACTION_QUEUE):
            builder.start("actions", {})
            for action in self.actions:
               builder.start("action", {})
               builder.data(str(action))
               builder.end("action")
            builder.end("actions")
        elif (self.type == Action.POSE):
            write_pose(self.pose)
        elif (self.type == Action.GRIPPER):
            builder.start("gripper", {})
            builder.start("is_open", {})
            builder.data(str(self.is_open))
            builder.end("is_open")
            builder.start("arm_index", {})
            builder.data(str(self.arm_index))
            builder.end("arm_index")
            builder.end("gripper")
        elif (self.type == Action.TRAJECTORY):
            for pose in self.poses:
                write_pose(pose)
    
    @staticmethod
    def get_saved_actions():
        return map(Action, 
            filter(lambda f: f.endswith(Action.FILE_EXTENSION),
                filter(isfile, 
                    map(lambda f: join(Action.ACTION_DIRECTORY, f), 
                        listdir(Action.ACTION_DIRECTORY)))))
        
    
    def save(self):
        '''saves action to file'''
        builder = TreeBuilder()
        builder.start("action", { "id" : str(self.id), "type" : str(self.type) })
        builder.start("name", {})
        builder.data(self.name)
        builder.end("name")
        self._write_type(builder)
        builder.end("action")
        doc = ElementTree(builder.close())
        doc.write(self.get_file(self.id))
    