FROM ubuntu:22.04

# Install required apt packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip

# Clone the repositories
COPY . /spot-rl
WORKDIR /spot-rl

COPY external /spot-rl/external
WORKDIR /spot-rl/external

# Put spot-private-sdk wheels in spot-rl/external/spot-python-sdk/prebuilt
# ^ This can be done with a `gitman update` or by manual intervention
# Install Python dependencies for low level spot API
WORKDIR /spot-rl/external/spot-python-sdk/prebuilt
RUN pip3 install bosdyn_api-4.0.0-py3-none-any.whl \
                bosdyn_core-4.0.0-py3-none-any.whl \
                bosdyn_client-4.0.0-py3-none-any.whl

RUN pip3 install pygame \
                pyPS4Controller \
                spatialmath-python \
                onnxruntime

# Copy the entrypoint script to the container
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint script as the entrypoint
ENTRYPOINT ["/entrypoint.sh"]