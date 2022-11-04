class Command {
    
    constructor(port){
        this.port = port
    }

    _checkStep = (step) => isNaN(step) ? 1 : parseInt(step)

    getLegAngle = () => this.port.write(`[get_leg 0]`)
    getLegSite = () => this.port.write(`[get_leg 1]`)
    getLegCalib = () => this.port.write(`[get_leg 2]`)
    getConfig = () => this.port.write(`[get_cfg]`)

    getUltrasonic = (refresh) => this.port.write(`[get_us ${refresh ? 1 : 0 }]`)

    setLegAngle = (leg, coxa, femur, tibia) => this.port.write(
            `[set_leg 0 ${leg} ${coxa} ${femur} ${tibia}]`)
    setLegSite = (leg, x, y, z) => this.port.write(
            `[set_leg 1 ${leg} ${x} ${y} ${z}]`)
    setLegCalib = (leg, coxa, femur, tibia) => this.port.write(
            `[set_leg 2 ${leg} ${coxa} ${femur} ${tibia}]`)
    
    gaitSit = () => this.port.write(`[gait_act 0]`)
    gaitStand = () => this.port.write(`[gait_act 1]`)

    gaitMoveForward = (step) => this.port.write(`[gait_act 2 ${this._checkStep(step)}]`)
    gaitMoveBackward = (step) => this.port.write(`[gait_act 3 ${this._checkStep(step)}]`)
    gaitTurnLeft = (step) => this.port.write(`[gait_act 4 ${this._checkStep(step)}]`)
    gaitTurnRight = (step) => this.port.write(`[gait_act 5 ${this._checkStep(step)}]`)
}

module.exports = { Command }