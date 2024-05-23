# Spot-RL
Code & Dockerfile for Spot Reinforcement Learning demo

# Import our image from .tar
```bash
docker load -i spot-rl-demo-<arch>.tar
docker tag spot-rl-demo:<arch> spot-rl-demo:latest
```

# Default Model 15k Steps
```bash
docker run --privileged --rm -it -v /dev/input:/dev/input spot-rl-demo:latest <ip of robot api> /spot-rl/external/models/
````

# Bring your own model (don't forget to set the IP)
```bash
docker run --privileged --rm -it -v /dev/input:/dev/input -v /path/to/folder/with/onz:/models spot-rl-demo:latest 192.168.x.y /models
```

# Example with local directory ./Model_Under_Test (don't forget to set the IP)
```bash
docker run --privileged --rm -it -v /dev/input:/dev/input -v ./Model_Under_Test/:/mut spot-rl-demo:latest 192.168.x.y /mut
```

# Installing without docker from locally cloned repo
```bash
sudo apt update
sudo apt install python3-pip
pip3 install gitman
gitman update
cd external/spot_python_sdk/prebuilt
pip3 install bosdyn_api-4.0.0-py3-none-any.whl
pip3 install bosdyn_core-4.0.0-py3-none-any.whl
pip3 install bosdyn_client-4.0.0-py3-none-any.whl
pip3 install pygame
pip3 install pyPS4Controller
pip3 install spatialmath-python
pip3 install onnxruntime
```
