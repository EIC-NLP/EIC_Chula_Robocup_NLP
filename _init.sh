# to run use "source init.sh"

# activating the virtual environment
conda activate nlp

# update pip to the latest version
pip install --upgrade pip

# pip install everything and setup python
source _pipinstall.sh

# run a script to generate config file and socket file with the correct paths
python _initalisation.py


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


