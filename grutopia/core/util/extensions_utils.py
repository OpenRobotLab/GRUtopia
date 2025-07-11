def dump_extensions():

    extensions = {}
    from grutopia.core.robot.controller import BaseController

    extensions['controllers'] = BaseController.controllers
    from grutopia.core.util.interaction import BaseInteraction

    extensions['interactions'] = BaseInteraction.interactions
    from grutopia.core.task.metric import BaseMetric

    extensions['metrics'] = BaseMetric.metrics
    from grutopia.core.object.object import BaseObject

    extensions['objs'] = BaseObject.objs
    from grutopia.core.robot.robot import BaseRobot

    extensions['robots'] = BaseRobot.robots
    from grutopia.core.sensor.sensor import BaseSensor

    extensions['sensors'] = BaseSensor.sensors
    from grutopia.core.task import BaseTask

    extensions['tasks'] = BaseTask.tasks
    return extensions


def reload_extensions(extensions):

    from grutopia.core.robot.controller import BaseController

    BaseController.controllers = extensions['controllers']
    from grutopia.core.util.interaction import BaseInteraction

    BaseInteraction.interactions = extensions['interactions']
    from grutopia.core.task.metric import BaseMetric

    BaseMetric.metrics = extensions['metrics']
    from grutopia.core.object.object import BaseObject

    BaseObject.objs = extensions['objs']
    from grutopia.core.robot.robot import BaseRobot

    BaseRobot.robots = extensions['robots']
    from grutopia.core.sensor.sensor import BaseSensor

    BaseSensor.sensors = extensions['sensors']
    from grutopia.core.task import BaseTask

    BaseTask.tasks = extensions['tasks']
