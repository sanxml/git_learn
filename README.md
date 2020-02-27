## git_test
### 用于学习使用git


#### 基本指令

git add 添加进仓库

git commit -m 添加修改了哪些内容

git status 显示工作区状态

git log 查看日志（git log --pretty=oneline）

git reflog 查看历史记录

git reset 恢复到某个版本（git reset --hard）


#### 线上仓库指令

git clone 克隆线上仓库到一个新目录（http协议或者ssh协议）

git push 提交到线上仓库的操作（修改权限，在.git/config里http://账号：密码@github.com）

git pull 拉取线上仓库到本地

ssh-keygen -t rsa -C "sanxml@foxmail.com" 创建公钥

复制 ~/.ssh/id_rsa.pub 到github，上传公钥


#### 分支操作

git branch 查看分支

git branch 分支名 添加分支

git checkout 分知名 切换分支

git branch -d 删除分支

git merge 分支名 合并分支


#### 忽略文件操作

新建.gitignore，编写规则
忽略文件上传，常用于不做更改的文件，如图片等


git --help 查看帮助
