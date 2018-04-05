cd /home/pi/Desktop/robotOne
h | grep -E "python|sudo" | awk '{print $2}' | sudo xargs kill
git reset --hard
git pull origin master
sudo python robot/main.py
