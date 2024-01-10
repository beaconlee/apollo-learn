

## 源码安装方式

启动环境容器：

```bash
bash docker/scripts/dev_start.sh
```

进入环境容器：

```bash
bash docker/srcipts/dev_into.sh
```

启动 Dreamview+：

```bash
bash scripts/bootstarp.sh start_plus
```

## 包管理安装方式：

启动 Apollo 环境容器：

```bash
aem start
```

进入 Apollo 环境容器：

```bash
aem enter
```

启动 Dreamview+

```bash
aem bootstrap start --plus
```



通过命令播放数据包(源码安装和包管理安装方式相同)：

```bash
cyber_recorder play -f ~/.apollo/resources/records/数据包名称 -l
// -l 是循环播放
```


