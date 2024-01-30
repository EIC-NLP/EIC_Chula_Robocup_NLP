conda create -n "nlp" python=3.9.16
conda activate nlp
pip install --upgrade pip

# Install pytorch with acceleration
echo "Installing Pytorch for Ubuntu with CUDA 12.1"
conda install pytorch==1.13.1 torchvision==0.14.1 torchaudio==0.13.1 pytorch-cuda=11.7 -c pytorch -c nvidia

# Unsure if works
conda install pyaudio

# Install all the requirements
pip install -r requirements.txt


# install the client package for nlp
cd src_client_pkg
pip install -e .

# change directory back to the root
cd ../

# text to tell the user
echo """\033[01;32mInstallation successful! please run the following command to start the server"""
echo ">>> python main.py \033[39m"

# rasa train on your computer
echo "Do you want to train rasa?, this is needed for first time setup. (yes/no)"
read yesorno
if [ "$yesorno" = yes ]; then
    echo "Training rasa..."
    cd rasa
    rasa train
    cd ../

else
    echo "Not training rasa"
fi

