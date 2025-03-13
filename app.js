const fs = require('fs');
const { SerialPort } = require('serialport')
const { ByteLengthParser } = require('@serialport/parser-byte-length')

const port = new SerialPort({ path: 'COM8', baudRate: 1500000 })

port.on('open',()=>{
    //send the command "write on speed 3 "
    port.write(Buffer.from([0x83]));
})

let fileStream = fs.createWriteStream('./stream1.bin')

const parser = port.pipe(new ByteLengthParser({ length: 8192 }))
parser.on('data', onPortData)
function onPortData(data) {
   //write a chunk of bitstream data into file
    fileStream.write(data)
}


setInterval(() => {
    const memoryUsage = process.memoryUsage();
    console.log(`RSS: ${memoryUsage.rss / 1024 / 1024} MB`);
    console.log(`Heap Total: ${memoryUsage.heapTotal / 1024 / 1024} MB`);
    console.log(`Heap Used: ${memoryUsage.heapUsed / 1024 / 1024} MB`);
    console.log(`External: ${memoryUsage.external / 1024 / 1024} MB`);
  }, 5000); 