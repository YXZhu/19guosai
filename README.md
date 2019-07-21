# 19guosai
19年国赛

# git pull
当你想要 push 你的新提交时，发现远程仓库在你上次拉取以后已经又有了改变，
也就是说你的新 commit 是基于旧提交的修改，这种情况下 Git 是不允许你进行 push 操作的，
你需要使自己的工作基于远程的提交

# git add xx
git add xx命令可以将xx文件添加到暂存区， 
git add -A . 一次添加所有改变的文件。注意 -A 选项后面还有一个句点。 
git add -A 表示添加所有内容， 
git add . 表示添加新文件和编辑过的文件不包括删除的文件
git add -u 表示添加编辑或者删除的文件，不包括新添加的文件。

# git commit
git commit 将暂存区里的改动给提交到本地的版本库
git commit -m “message” message 简要说明这次提交内容的语句
git commit --amend 不增加一个新的commit-id的情况下将新修改的代码追加到前一次的commit-id中

# git push
负责将你的变更上传到指定的远程仓库。远程仓库中的 master 分支被更新，
我们的远程分支（o/master）也同样会被更新。
