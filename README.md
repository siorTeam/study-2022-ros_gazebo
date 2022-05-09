2022 ROS_GAZEBO
===============

# git clone 사용법

## 1) git 설치

       sudo apt-get install git

## 2) ssh 키 등록

       cd ~/.ssh

       ssh-keygen

       cat ~/.ssh/id_rsa.pub

cat 해서 뜬 글 복사 후 Gitlab의 자기 프로파일 - preference - SSH Keys 에다 붙여넣고 기기 등록

## 3) 이 프로젝트 우측 위 파랑 clone 상자 클릭

Clone with ssh 링크 복사

       cd ~/catkin_ws/src

       git clone git@gitlab.com:skku-sior/study_ros/2022-ros_gazebo.git

## 4) 실행하고 싶은 브랜치로 이동

git checkout 이동하고 싶은 브랜치 이름 입력

       ex) git checkout 3rd-week---gazebo_one_joint

## 5) 이후 catkin_make 진행

       cd ~/catkin_ws/src
       
       catkin_make