cd /home/pi/Desktop/robotOne
ps auc | grep -E "python|sudo" | awk '{print $2}' | sudo xargs kill -9
git reset --hard
git pull origin master
sudo python robot/main.py
