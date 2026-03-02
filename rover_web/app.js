// ======== CONFIG ========
const ROSBRIDGE_URL = `ws://192.168.0.42:9090`;
document.getElementById('wsurl').textContent = ROSBRIDGE_URL;

// ======== ROS Connection ========
const dot = document.getElementById('dot');
const statusEl = document.getElementById('status');

const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

// ======== Topics ========
const cmdVel = new ROSLIB.Topic({
  ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist'
});
const resetOdom = new ROSLIB.Topic({
  ros, name: '/reset_odometry', messageType: 'std_msgs/Bool'
});

function publishCmd(v, w){
  cmdVel.publish(new ROSLIB.Message({
    linear: { x: v, y: 0.0, z: 0.0 },
    angular:{ x: 0.0, y: 0.0, z: w }
  }));
}
function stopNow(){ publishCmd(0.0, 0.0); }

// ======== Sliders ========
const vmax = document.getElementById('vmax');
const wmax = document.getElementById('wmax');
const hz = document.getElementById('hz');

const vmaxLabel = document.getElementById('vmaxLabel');
const wmaxLabel = document.getElementById('wmaxLabel');
const hzLabel = document.getElementById('hzLabel');

function syncLabels(){
  vmaxLabel.textContent = Number(vmax.value).toFixed(2);
  wmaxLabel.textContent = Number(wmax.value).toFixed(2);
  hzLabel.textContent = hz.value;
}
vmax.addEventListener('input', syncLabels);
wmax.addEventListener('input', syncLabels);
hz.addEventListener('input', syncLabels);
syncLabels();

// ======== HOLD Buttons ========
let holdTimer = null;

function stopHold(){
  if (holdTimer) clearInterval(holdTimer);
  holdTimer = null;
  stopNow();
}

// Joystick interval
let joyInterval = null;
let joyV = 0.0, joyW = 0.0;

function stopJoyPublish(){
  if (joyInterval) clearInterval(joyInterval);
  joyInterval = null;
}
function startJoyPublish(){
  stopJoyPublish();
  const periodMs = Math.round(1000 / Number(hz.value));
  joyInterval = setInterval(() => publishCmd(joyV, joyW), periodMs);
}

function startHold(v, w){
  stopJoyPublish(); // avoid conflict
  stopHold();
  const periodMs = Math.round(1000 / Number(hz.value));
  publishCmd(v, w);
  holdTimer = setInterval(() => publishCmd(v, w), periodMs);
}

function bindHold(btn, v, w){
  const V = () => Number(vmax.value) * v;
  const W = () => Number(wmax.value) * w;

  const down = (e) => { e.preventDefault(); startHold(V(), W()); };
  const up = (e) => { e.preventDefault(); stopHold(); };

  btn.addEventListener('mousedown', down);
  btn.addEventListener('mouseup', up);
  btn.addEventListener('mouseleave', up);

  btn.addEventListener('touchstart', down, {passive:false});
  btn.addEventListener('touchend', up, {passive:false});
  btn.addEventListener('touchcancel', up, {passive:false});
}

bindHold(document.getElementById('btnF'), +1.0,  0.0);
bindHold(document.getElementById('btnB'), -1.0,  0.0);
bindHold(document.getElementById('btnL'),  0.0, +1.0);
bindHold(document.getElementById('btnR'),  0.0, -1.0);

document.getElementById('btnStop').addEventListener('click', () => stopHold());
document.getElementById('btnReset').addEventListener('click', () => {
  resetOdom.publish(new ROSLIB.Message({ data: true }));
});

// ======== Joystick ========
const joyZone = document.getElementById('joy');
const manager = nipplejs.create({
  zone: joyZone,
  mode: 'static',
  position: { left: '50%', top: '50%' },
  color: 'white',
  size: 170,
  restOpacity: 0.35
});

manager.on('start', () => {
  stopHold(); // avoid conflict
  startJoyPublish();
});

manager.on('move', (evt, data) => {
  const maxV = Number(vmax.value);
  const maxW = Number(wmax.value);

  const vx = data.vector.x;
  const vy = data.vector.y;
  const forward = -vy;

  const dead = 0.10;
  const f = Math.abs(forward) < dead ? 0 : forward;
  const t = Math.abs(vx)      < dead ? 0 : vx;

  joyV = f * maxV;
  joyW = t * maxW;
});

manager.on('end', () => {
  stopJoyPublish();
  joyV = 0.0; joyW = 0.0;
  stopNow();
});

// ======== Safety ========
ros.on('connection', () => {
  statusEl.textContent = "connected";
  dot.className = "dot ok";
});
ros.on('error', (e) => {
  statusEl.textContent = "error";
  dot.className = "dot bad";
  console.log(e);
  stopHold();
  stopJoyPublish();
  stopNow();
});
ros.on('close', () => {
  statusEl.textContent = "closed";
  dot.className = "dot bad";
  stopHold();
  stopJoyPublish();
  stopNow();
});

// ======== Telemetry subs ========
const leftTicks = new ROSLIB.Topic({ ros, name:'/left_ticks', messageType:'std_msgs/Int32' });
const rightTicks = new ROSLIB.Topic({ ros, name:'/right_ticks', messageType:'std_msgs/Int32' });
const distance = new ROSLIB.Topic({ ros, name:'/distance', messageType:'std_msgs/Float32' });
const odom = new ROSLIB.Topic({ ros, name:'/odom', messageType:'nav_msgs/Odometry' });

leftTicks.subscribe(m => { document.getElementById('lt').textContent = m.data; });
rightTicks.subscribe(m => { document.getElementById('rt').textContent = m.data; });
distance.subscribe(m => { document.getElementById('dist').textContent = Number(m.data).toFixed(3); });

odom.subscribe(m => {
  document.getElementById('x').textContent = m.pose.pose.position.x.toFixed(3);
  document.getElementById('y').textContent = m.pose.pose.position.y.toFixed(3);

  const q = m.pose.pose.orientation;
  const siny_cosp = 2*(q.w*q.z + q.x*q.y);
  const cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  document.getElementById('yaw').textContent = yaw.toFixed(3);

  document.getElementById('v').textContent = m.twist.twist.linear.x.toFixed(3);
  document.getElementById('w').textContent = m.twist.twist.angular.z.toFixed(3);
});

// ======== Camera refresh (anti-freeze) ========
setInterval(() => {
  const cam = document.getElementById('cam');
  if (cam) cam.src = "http://192.168.0.42:8080/?action=stream&t=" + Date.now();
}, 15000);
