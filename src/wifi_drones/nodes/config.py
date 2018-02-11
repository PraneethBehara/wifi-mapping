turtlebot = dict(
    pub = dict(
        odom = '/odom'
    )
)

rtabmap = dict(
    pub = dict(
        grid_map = '/map'
    ,   proj_map = '/rtabmap/proj_map'
    )
)

move_base = dict(
    pub = dict(
        status = '/move_base/status'
    )
,   srv  = dict(
        make_plan = '/move_base/make_plan'
    )
)

map_combiner = dict(
    name = 'map_combiner'
,   pub  = dict(
        map_explore = 'map_explore'
    )
)

bot_mover = dict(
    name = 'bot_mover'
,   pub  = dict(
        cmd_vel     = '/cmd_vel_mux/input/navi'
    ,   move_goal   = '/move_base_simple/goal'
    ,   ready_state = '/ready_state'
    )
,   srv  = dict(
        move_bot = 'move_bot'
    )
)

controller = dict(
    name = 'controller'
,   pub  = dict(
        viz_goal = 'viz_goal'
    ,   viz_aps = 'viz_aps'
    ,   show_frontiers = 'show_frontiers'
    ,   show_other_frontiers = 'show_other_frontiers'
    )
)

costmaps = dict(
	name = 'costmaps'
,	pub = dict(
  		costmap = '/move_base/local_costmap/costmap'
  	)
)

sim_envs = dict(
    name = 'aplocs'
,   g_truth = {'apid':[ 1  ,     2,    3,     4,    5,     6,     7,      8,      9,   10,    11,    12,   13]
             , 'x'   :[ 0  ,  16.5, 28.0,  48.0, 39.5,  24.0,   13.0,     0,      0,    0,  12.0,  12.0, 12.0]
             , 'y'   :[ 1.0,   0.0,    0, -14.0, -15.0, -15.0, -15.0,  9.25,  20.85, 38.0,  47.0,  31.0,  11.0]
             , 'z'   :[ 3  ,     3,    3,     3,     3,     3,     3,     3,      3,    3,     3,     3,    3]}
,   unscaled= {'apid':[  1,     2,     3,     4,     5,     6,     7,    8,     9,   10,    11,    12,    13]
             , 'x'   :[1.0,   0.0,     0, -14.0, -15.0, -15.0, -15.0, 9.25, 20.85, 38.0,  47.0,  31.0,  11.0]
             , 'y'   :[  0, -16.5, -28.0, -48.0, -39.5, -24.0, -13.0,    0,     0,    0, -12.0, -12.0, -12.0]
             , 'z'   :[  3,     3,   3,     3,     3,     3,     3,     3,     3,    3,    3,    3,    3]}
,   testenv = {'apid':[ 1, 2    , 3 ,      4,    5,     6,     7,    8,   9,   10 ]
               , 'x' :[ 25, 0.707, 20 , -7.0, -11.0,  3.0,  22.0, 20.0, 1.0,  -8.0]
               , 'y' :[ 10, 0.707, 10 , -14.0, 10.0, -5.0, -25.0, 19.0, 20.0,  7.0]
               , 'z' :[ 3,     3,   3 ,     3,    3,    3,     3,    3,    3,   3 ]}
,   testenv4= {'apid':[ 1, 2    , 3 ,      4]
               , 'x' :[ 3.0, 0.707, 8.0 , -7.0]
               , 'y' :[ -5.0, 0.707, 9.0 , -8.0]
               , 'z' :[ 3,     3,   3 ,     3]}
,   testenv1={'apid': [1]
               , 'x': [3.0]
               , 'y': [-5.0]
               , 'z': [3]}
,   testenv5={'apid': [1, 2, 3, 4]
               , 'x': [5.0, -5.0, -5.0, 5.0]
               , 'y': [-5.0, -5.0, 5.0, 5.0]
               , 'z': [3, 3, 3, 3]}
,     rect= {'apid': [1, 2, 3, 4]
               , 'x': [3.0,  3.0, 15.0, 15.0]
               , 'y': [3.0, 30.0,  3.0, 30.0]
               , 'z': [3, 3, 3, 3, 3]}
,     square= {'apid': [1, 2, 3, 4]
               , 'x': [2.0, 15.0,  2.0, 15.0]
               , 'y': [2.0,  2.0, 15.0, 15.0]
               , 'z': [3, 3, 3, 3]}
,        wg= {'apid': [1, 2, 3, 4]
               , 'x': [2.0, 20.0, 20.0,  2.0]
               , 'y': [2.0, 20.0,  2.0, 20.0]
               , 'z': [3, 3, 3, 3]}
)

load_env = sim_envs['rect']