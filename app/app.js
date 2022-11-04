const { app, BrowserWindow } = require('electron')
const path = require('path')
const url = require('url')

let mainWindow

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 800,
        height: 600,
        minWidth: 800,
        minHeight: 600,
        show: false,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false,
            // preload: path.join(__dirname, 'preload.js')
        }
    })
    mainWindow.maximize()
    mainWindow.show()
    mainWindow.loadURL(url.format({
        pathname: path.join(__dirname, './src/index.html'),
        protocol: 'file:',
        slashes: true
    }))
    mainWindow.webContents.openDevTools()
    mainWindow.on('closed', function() {
        mainWindow = null
    })
}
app.allowRendererProcessReuse = false
app.on('ready', createWindow)
app.on('window-all-closed', function() {
    app.quit()
})
app.on('activate', function() {
    if (mainWindow === null) {
        createWindow()
    }
})