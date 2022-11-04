const { SerialPort, ReadlineParser } = require('serialport')
const { createApp } = require('vue')
const { Command } = require('./command')
const { ListSerialPorts, TryParseJSON } = require('./helper')
const {
    BaudRateList, Tabs, UltrasonicTitle,
    LegTitle, ConfigTitle, UltrasonicUpdateInterval,
} = require('./constant')

var Port = undefined
var Parser = undefined
var Cmd = undefined

createApp({
    data() {
        return {
            active_tab: "",
            serial: {
                active: {},
                is_open: false,
                baud_rate: 115200,
                update_interval: 3000,
                list: [],
                log: [],
                input_field: "",
            },
            ultrasonic: {
                distance: [...Array(UltrasonicTitle.length).fill(0)],
                enable_auto_update: true,
                update_interval: 1000,
                update_timer: undefined,
            },
            site: [...Array(4)].map(_ => Array(3).fill(0)),
            servo: [...Array(4)].map(_ => Array(3).fill(0)),
            is_apply_calib: true,
            gait_step: 1,
            config: {
                servo_calib: [...Array(4)].map(_ => Array(3).fill(0)),
                coxa_len: 0,
                femur_len: 0,
                tibia_len: 0,

                z_base: 0,
                z_stand: 0,
                z_up: 0,
                x_base: 0,
                x_offset: 0,
                y_base: 0,
                y_step: 0,

                move_speed: 0,
                stand_seat_speed: 0,
                leg_move_speed: 0,
                body_move_speed: 0,
                spot_turn_speed: 0,
            }
        };
    },

    methods: {
        listSerial() {
            ListSerialPorts().then(data => {
                this.serial.list = data
            })
        },
        openSerial(serial) {
            if (this.serial.is_open) return
            const { path } = serial
            const baudRate = this.serial.baud_rate
            Port = new SerialPort({ path, baudRate, autoOpen: true })
            Port.on('close', this.serialOnClose)
            Port.on('open', this.serialOnOpen)
            this.serial.active = serial
        },
        closeSerial() {
            Port.close()
        },
        serialOnOpen() {
            console.log('Port Opened')
            Parser = new ReadlineParser()
            Parser.on('data', this.serialOnMessage)
            Port.pipe(Parser)
            Cmd = new Command(Port)
            this.serial.is_open = true
            this.changeUsAutoUpdate(false)
            this.refreshValues()
        },
        serialOnClose() {
            console.log('Port Closed')
            Port = undefined
            Parser = undefined
            Cmd = undefined
            this.serial.is_open = false
            this.changeUsAutoUpdate(false)
            this.active_tab = this.Tabs[0]
        },
        serialOnMessage(msg) {
            msg = msg + ""
            var obj = TryParseJSON(msg)
            if (obj === undefined) console.log(msg)
            else this.parseObjMessage(obj)
        },
        serialSend(cmd) {
            if (!this.serial.is_open) return
            const is_str = typeof (cmd) == 'string'
            const input = is_str ? cmd : this.serial.input_field
            if (input.length < 1) return

            console.log('->' + input)
            Port.write(input)

            const is_unique = this.serial.log.indexOf(input) < 0
            if (is_unique)
                this.serial.log.push(input)
            if (this.serial.log.length > 10)
                this.serial.log.shift()
        },
        parseObjMessage(obj) {
            console.log(obj)
            const {
                site, angle, calib, us,
                coxa_len, femur_len, tibia_len,
                z_base, z_stand, z_up, x_base, x_offset, y_base, y_step,
                move_speed, stand_seat_speed, leg_move_speed,
                body_move_speed, spot_turn_speed,
            } = obj
            for (i = 0; i < 4; i++) {
                for (j = 0; j < 3; j++) {
                    if (angle !== undefined)
                        this.servo[i][j] = angle[i][j]
                    if (site !== undefined)
                        this.site[i][j] = site[i][j]
                    if (calib !== undefined)
                        this.config.servo_calib[i][j] = calib[i][j]
                }
            }

            if (us !== undefined && Array.isArray(us))
                this.ultrasonic.distance = [...us]
            if (coxa_len) this.config.coxa_len = coxa_len
            if (femur_len) this.config.femur_len = femur_len
            if (tibia_len) this.config.tibia_len = tibia_len
            if (z_base) this.config.z_base = z_base
            if (z_stand) this.config.z_stand = z_stand
            if (z_up) this.config.z_up = z_up
            if (x_base) this.config.x_base = x_base
            if (x_offset) this.config.x_offset = x_offset
            if (y_base) this.config.y_base = y_base
            if (y_step) this.config.y_step = y_step
            if (move_speed) this.config.move_speed = move_speed
            if (stand_seat_speed) this.config.stand_seat_speed = stand_seat_speed
            if (leg_move_speed) this.config.leg_move_speed = leg_move_speed
            if (body_move_speed) this.config.body_move_speed = body_move_speed
            if (spot_turn_speed) this.config.spot_turn_speed = spot_turn_speed

        },
        refreshValues(params) {
            if (!this.serial.is_open) return
            const cmd_list = [
                { key: 'config', cmd: () => Cmd.getConfig() },
                { key: 'servo', cmd: () => Cmd.getLegAngle() },
                { key: 'site', cmd: () => Cmd.getLegSite() },
                { key: 'calib', cmd: () => Cmd.getLegCalib() },
                { key: 'ultrasonic', cmd: () => Cmd.getUltrasonic(true) },
            ]
            const command = (k) => {
                cmd_list.forEach(e => {
                    if (k == e.key) e.cmd()
                })
            }
            if (Array.isArray(params)) params.forEach(e => command(e))
            else if (typeof (params) === 'string') command(params)
            else cmd_list.forEach(e => command(e.key))
        },
        siteDefault(leg) {
            if (!this.serial.is_open) return
            if (leg < 0 || leg > 3) return
            leg = parseInt(leg)

            this.site[leg][0] = this.config.x_base
            this.site[leg][1] = this.config.y_base
            this.site[leg][2] = this.config.z_base

            Cmd.setLegSite(leg, this.site[leg][0], this.site[leg][1], this.site[leg][2])
        },
        servoDefault(leg) {
            if (!this.serial.is_open) return
            if (leg < 0 || leg > 3) return
            leg = parseInt(leg)

            this.servo[leg][0] = 90
            this.servo[leg][1] = 90
            this.servo[leg][2] = 90

            if (this.is_apply_calib) {
                this.servo[leg][0] += this.config.servo_calib[leg][0]
                this.servo[leg][1] += this.config.servo_calib[leg][1]
                this.servo[leg][2] += this.config.servo_calib[leg][2]
            }
            Cmd.setLegAngle(leg, this.servo[leg][0], this.servo[leg][1], this.servo[leg][2])
        },
        gaitAction(action, step) {
            if (!this.serial.is_open) return

            step = typeof (step) == 'number' ? step : this.gait_step
            step = isNaN(parseInt(step)) ? 1 : parseInt(step)

            if (action === 'sit') Cmd.gaitSit()
            if (action === 'stand') Cmd.gaitStand()
            if (action === 'move_forward') Cmd.gaitMoveForward(step)
            if (action === 'move_backward') Cmd.gaitMoveBackward(step)
            if (action === 'turn_left') Cmd.gaitTurnLeft(step)
            if (action === 'turn_right') Cmd.gaitTurnRight(step)
        },
        applyServoAngle(leg) {
            if (!this.serial.is_open) return
            if (leg < 0 || leg > 3) return

            let coxa = parseInt(this.servo[leg][0])
            let femur = parseInt(this.servo[leg][1])
            let tibia = parseInt(this.servo[leg][2])

            if (this.is_apply_calib) {
                coxa += this.config.servo_calib[leg][0]
                femur += this.config.servo_calib[leg][1]
                tibia += this.config.servo_calib[leg][2]
            }
            Cmd.setLegAngle(leg, coxa, femur, tibia)
        },
        applySite(leg) {
            if (!this.serial.is_open) return
            if (leg < 0 || leg > 3) return

            const x = parseInt(this.site[leg][0])
            const y = parseInt(this.site[leg][1])
            const z = parseInt(this.site[leg][2])

            Cmd.setLegSite(leg, x, y, z)
        },
        applyConfig(cfg) {
            if (typeof (cfg) != 'object') return
            if (cfg.id == undefined) return

            if (cfg.type != 'table') {
                if (!Array.isArray(cfg.config)) return
                const configs = {}
                cfg.config.forEach(c => {
                    if (this.config[c.id] !== undefined)
                        configs[c.id] = this.config[c.id]
                })
                for (k in configs) {
                    const cmd = `[${k} ${configs[k]}]`
                    console.log(`->${cmd}`)
                    Port.write(cmd)
                }
            } else if (cfg.id == 'servo_calib') {
                for (i = 0; i < 4; i++) {
                    const coxa = parseInt(this.config.servo_calib[i][0])
                    const femur = parseInt(this.config.servo_calib[i][1])
                    const tibia = parseInt(this.config.servo_calib[i][2])

                    Cmd.setLegCalib(i, coxa, femur, tibia)
                }
            }
        },
        changeUsAutoUpdate(state) {
            if (typeof (state) !== 'boolean') return

            this.ultrasonic.enable_auto_update = state
            const update_func = () => {
                if (this.serial.is_open && this.active_tab == 'sensors')
                    this.refreshValues('ultrasonic')
            }
            if (state) {
                this.ultrasonic.update_timer = setInterval(
                    update_func, this.ultrasonic.update_interval)
            } else {
                clearInterval(this.ultrasonic.update_timer)
            }
        },
        removeLog(index) {
            if (index < this.serial.log.length)
                this.serial.log.splice(index, 1)
        },
    },
    computed: {
        Tabs,
        BaudRateList,
        UltrasonicTitle,
        LegTitle,
        ConfigTitle,
        UltrasonicUpdateInterval,
    },
    mounted() {
        this.listSerial()
        setInterval(this.listSerial, this.serial.update_interval)
        this.active_tab = 'connection'
    }
}).mount("#app")