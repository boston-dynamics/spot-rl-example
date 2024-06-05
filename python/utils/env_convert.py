import json
import os
import yaml

def remove_slice(dictionary):
    for key, value in dictionary.items():
        if type(value) is dict:
            remove_slice(value)
        else:
            if "slice" in str(value):  
              dictionary[key] = None
    return dictionary


def load_local_cfg(resume_path: str) -> dict:
    env_cfg_yaml_path = os.path.join(cfg_dir, "env.yaml")
    # load yaml
    with open(env_cfg_yaml_path) as yaml_in:
        env_cfg = yaml.load(yaml_in, Loader=yaml.Loader)

    env_cfg = remove_slice(env_cfg)
    return env_cfg

print("Enter the path to the env.yaml directory")
cfg_dir = input()
env_cfg = load_local_cfg(cfg_dir)

cfg_save_path = os.path.join(cfg_dir, "env_cfg.json")
with open(cfg_save_path, "w") as fp:
    json.dump(env_cfg, fp, indent=4)