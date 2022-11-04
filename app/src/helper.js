const { SerialPort, ReadlineParser } = require('serialport')

const ListSerialPorts = async () => {
    return await SerialPort.list().then((ports, err) => {
        if (err) {
            console.error(err.message)
            return
        }           
        if (ports.length === 0)
            console.error('No ports discovered')
        return ports.map(i => {
            const { path, friendlyName, manufacturer } = i
            return { path, friendlyName, manufacturer }
        })
    })
}
const IsHasUnicode = (str) => {
    for (var i = 0, n = str.length; i < n; i++) {
        if (str.charCodeAt(i) > 127)
            return true; 
    }
    return false;
}
const TryParseJSON = (str) => {
    if (str.charAt(0) == '{'){
        try {
            const obj = JSON.parse(str)
            return obj
        } catch { }
    }
    return undefined
}

module.exports = { 
    ListSerialPorts, 
    IsHasUnicode,
    TryParseJSON,
}