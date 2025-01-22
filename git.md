# git使用简介

参考：[菜鸟教程github](https://www.runoob.com/w3cnote/git-guide.html)

# 1 本地配置

1. 本地创建ssh key
   
```bash
$ ssh-keygen -t rsa -C "your_email@youremail.com"
```

2. 进入`~/.ssh/id_rsa.pub`，复制文件中的内容。

3. 进入github账户配置（Account Settings），添加SSH key，粘贴复制的key。

4. 在终端中输入如下命令，测试是否验证成功。
   
```bash
$ ssh -T git@github.com
```

输入yes，若验证成功，会看到You've successfully authenticated, but GitHub does not provide shell access.

5. 设置用户名和邮箱
   
```bash
$ git config --global user.name "your name"
$ git config --global user.email "your_email@youremail.com"
```

设置完成后，即可向github远程仓库上传自己的项目。

# 2 项目管理

## 2.1 克隆仓库

克隆本地仓库：

```bash
$ git clone /path/to/repository 
```

克隆远程仓库：

```bash
$ git clone username@host:/path/to/repository
```

## 2.2 工作流

本地git仓库由三颗树组成：工作目录 Working dir，存放实际项目文件；暂存区 Index，保存项目临时改动；HEAD，指向最后一次提交的内容。

可以使用`add`**提出更改**将工作目录中的文件添加到暂存区，命令如下：

```bash
$ git add <filename>
```

使用`commit`**提交实际改动**，会将暂存区的文件提交到HEAD。这时还没有将项目提交到远程仓库。

```bash
$ git commit -m "代码提交信息"
```

通过`branch`**建立分支**。

```bash
$ git branch -M main
```

本地仓库需要通过`remote`**连接到远程仓库**。使用如下命令添加。main为分支名称。

```bash
$ git remote add origin <server>
```

使用`push`**推送改动**。将HEAD中项目提交到远程仓库。

```bash
$ git push -u origin main
```

使用`checkout`切换分支。

```bash
$ git checkout
```

# 示例

**初始化并上传一个本地仓库**

```bash
$ echo "# project" >> README.md
$ git init
$ git add README.md
$ git commit -m "first commit"
$ git branch -M main
$ git remote add origin git@github.com:username/repository.git
$ git push -u origin main
```

2023.6 重装系统后，没有办法 push。解决方案：

```shell
git remote -v
git remote set-url origin git@github.com:USERNAME/REPONAME.git
```
