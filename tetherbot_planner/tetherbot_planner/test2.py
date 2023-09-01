from tbotlib import TbTetherbot, TetherbotVisualizer, TransformMatrix

tbot: TbTetherbot = TbTetherbot.load('/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot.pkl')


#print(tbot.platform.T_world.decompose())
#print(tbot.stability())
tbot.pick(2, correct_pose=True)
tbot.platform.T_local = TransformMatrix([0.6, 1.1, 0.068, 0, 0, 95])
#print(tbot.platform.T_world.decompose())
print(tbot.stability())
vi = TetherbotVisualizer(tbot)
vi.run()