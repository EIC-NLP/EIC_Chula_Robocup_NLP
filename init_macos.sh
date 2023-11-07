
# Building NLP Enviroment
conda create -n "nlp" python=3.9.16
conda activate nlp
pip install --upgrade pip

# Install pytorch with acceleration
echo "Installing Pytorch for Mac..."
# install pytorch with acceleration
conda install pytorch-nightly::pytorch torchvision torchaudio -c pytorch-nightly
echo "Finished installing Pytorch for Mac"

# pyaudio requires portaudio by homebrew, cannot be install by only pip
brew install portaudio
pip install pyaudio

# custom library for coloring the terminal and clearing
pip install git+https://github.com/GameTL/ratfingers.git --use-pep517

# Install all the requirements
pip install -r requirements.txt


# install the client package for nlp
cd src_client_pkg
pip install -e .

# change directory back to the root
cd ../

# Building Rasa Enviroment
conda create -n "rasa" python=3.9.16
conda activate nlp
pip install --upgrade pip
pip install rasa
pip install fbmessenger==6.0.0
pip install SQLAlchemy==1.4.49

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




