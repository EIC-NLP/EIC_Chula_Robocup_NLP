# to run use "source pipinstall.sh"

# activating the virtual environment
conda activate nlp


# Install pytorch with acceleration

if [[ "$(uname)" == "Darwin" ]]; then
    # commands to run on Mac
    echo "Installing Pytorch for Mac"
    # install pytorch with acceleration
    conda install pytorch-nightly::pytorch torchvision torchaudio -c pytorch-nightly

elif [[ "$(uname)" == "Linux" ]]; then
    # commands to run on Linux
    echo "Installing Pytorch for Ubuntu with CUDA 12.1"
    # install pytorch with acceleration
    conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch-nightly -c nvidia
else
    echo "Unsupported operating system"
fi

# install pyaudio, pip fails
# conda install -c anaconda pyaudio
# pip install pipwin
# pipwin install pyaudio

conda install pyaudio
pip install git+https://github.com/GameTL/ratfingers.git --use-pep517
# pip install all the requirements
pip install -r requirements.txt



# install the client package for nlp
cd src_client_pkg
pip install -e .

# change directory back to the root
cd ../

# text to tell the user
echo """\033[01;32mInstallation successful! please run the following command to start the server"""
echo ">>> python main.py \033[39m"