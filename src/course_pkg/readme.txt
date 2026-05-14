机器人导航仿真功能包包名	course_pkg
course_pkg下的文件夹：
	config:		存放yaml配置文件，用于频繁修改的自定义参数，且不需要重新编译功能包
	include:		存放头文件
	launch:		存放launch文件，用于启动 ROS 节点、加载参数和启动仿真环境。
	meshes:		存放机器人模型渲染文件
	src:			存放源文件
	urdf:			存放Unified Robot Description Format文件，即机器人模型文件
	worlds:		存放机器人周边环境模型文件
	CmakeLists.txt:	用于定义功能包的编译规则。
	package.xml:		用于定义功能包的元数据，功能包名称、版本、描述、维护者等信息
