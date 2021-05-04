cd ~
mkdir -p hummingbird_ws/src
cd hummingbird_ws/src
catkin_init_workspace

git clone https://magiccvs.byu.edu/gitlab/hummingbird/hb_student.git
git clone https://magiccvs.byu.edu/gitlab/hummingbird/hb.git

cd hb
git checkout --track origin/fall_2020
cd ../hb_student
git checkout --track origin/fall_2020
cd ..

cd ..
catkin_make

source ~/hummingbird_ws/devel/setup.bash
echo "source ~/hummingbird_ws/devel/setup.bash" >> ~/.bashrc

echo "Please copy over your dynamics.py file now."
