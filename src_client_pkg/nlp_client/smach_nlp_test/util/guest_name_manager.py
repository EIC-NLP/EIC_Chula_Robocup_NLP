#!/usr/bin/env python
import yaml

class GuestNameManager:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.data_yaml = self.read_yaml()

    def read_yaml(self):
        with open(self.yaml_path, "r") as f:
            try:
                yaml_data = yaml.safe_load(f)
                for data in yaml_data:
                    if data is None:
                        yaml_data.remove(data)
                return yaml_data
            except yaml.YAMLError as exc:
                print(exc)
                return None
    def write_yaml(self, data):
        with open(self.yaml_path, 'w') as file:
            yaml.dump(data, file, encoding=('utf-8'))

    def get_guest_name(self,role):
        for data in self.data_yaml:
            if data["role"] == role:
                return data["name"]

    def get_guest_fav_drink(self,role):
        for data in self.data_yaml:
            if data["role"] == role:
                return data["fav_drink"]

    def add_guest_name(self, role, name):
        for data in self.data_yaml:
            if data["role"] == role:
                # already have this guest
                return None
        guest = {"name":str(name), "role":role} # there are no this guest yet
        self.data_yaml.append(guest)
        # print(yaml.dump(self.data_yaml))
        self.write_yaml(self.data_yaml)

    def add_guest_fav_drink(self, role, drink):
        for i, data in enumerate(self.data_yaml):
            if data["role"] == role:
                data["fav_drink"] = str(drink)
                self.data_yaml[i] = data
        self.write_yaml(self.data_yaml)

    def reset(self):
        host = {}
        for data in self.data_yaml:
            if data["role"] == "host":
                host = data
                self.data_yaml = [host]
                self.write_yaml(self.data_yaml)
        
if __name__ == "__main__":
    gm = GuestNameManager("../../config/receptionist_database.yaml")
    gm.add_guest_name("guest_3","earth")
    gm.add_guest_fav_drink("guest_3", "jack daniel")
    print(gm.get_guest_fav_drink("guest_3"))
    gm.reset()