# 学习使用github

## 已有仓库调用

### 1. git设置密钥

``` shell
ssh-keygen -t rsa -C "sanxml@foxmail.com"
```

### 2. 将公钥添加到仓库

``` shell
cat ~/.ssh/id_rsa.pub
```

将输出的公钥复制，添加到仓库中

![添加公钥](./assets/ssh.png)

### 3. 克隆仓库

``` shell
git clone https://github.com/sanxml/git_learn.git
```

### 4. 联系本地仓库和远程仓库

``` shell
git remote add origin git@github.com:sanxml/git_learn.git
```

### 5. 更改和提交

``` shell
git add README.md
git commit -m "update README.md"
```

### 6. 推送到远程仓库

``` shell
git push -u origin master //远程仓库空时
git push origin master
```
