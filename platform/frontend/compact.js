const fs = require('fs')
const repl =require('node:repl')
const path = require('path')


const msg = 'message';


const { spawn } = require('node:child_process');
const ls = spawn('ls', ['-lh', '/home/yum']);

ls.stdout.on('data', (data) => {
  console.log(`stdout: ${data}`);
});

ls.stderr.on('data', (data) => {
  console.error(`stderr: ${data}`);
});

ls.on('close', (code) => {
  console.log(`child process exited with code ${code}`);
});

repl.start('> ').context.m = msg;
