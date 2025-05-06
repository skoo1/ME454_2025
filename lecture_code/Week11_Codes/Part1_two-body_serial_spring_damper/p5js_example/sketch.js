// p5.js: simulate and visualize two-body system with prismatic joint in real time

let m1 = 0.20, m2 = 0.10;
let l1 = 0.50;   // half length of body 1
let l2 = 0.25;   // half length of body 2
let I1 = m1 * (2 * l1) ** 2 / 12;
let I2 = m2 * (2 * l2) ** 2 / 12;
let g = 9.81;

let K = 10.0;
let C = 2.0;
let L0 = 0.05;   // nominal length of the srping

let q = [];      // [x1, y1, theta1, x2, y2, theta2]
let q_dot = [];  // [vx1, vy1, w1, vx2, vy2, w2]
let M;           // mass matrix 6x6

let dt = 0.001;

function setup() {
  createCanvas(600, 600);
  frameRate(60);
  scale(100, -100);
  
  // Initial conditions
  let theta1 = Math.PI / 4;
  let theta2 = Math.PI / 4;
  let x1 = l1 * Math.sin(theta1);
  let y1 = -l1 * Math.cos(theta1);
  let x2 = (2 * l1 + L0 + l2) * Math.sin(theta2);
  let y2 = -(2 * l1 + L0 + l2) * Math.cos(theta2);
  q = [x1, y1, theta1, x2, y2, theta2]
  q_dot = [0, 0, 0, 0, 0, 0];
  M = math.diag([m1, m1, I1, m2, m2, I2]);
}

function draw() {
  background(255);
  translate(width / 2, height / 2);
  scale(100, -100);
  
  let simSteps = Math.floor((deltaTime / 1000) / dt);
  
  for (let i = 0; i < simSteps; i++) {
    step();  // perform high-frequency simulation steps
  }
  
  drawSystem(q);
}

function step() {
  let q_ddot = compute_q_ddot(q, q_dot);
  for (let i = 0; i < 6; i++) {
    q_dot[i] += q_ddot[i] * dt;
    q[i]     += q_dot[i] * dt;
  }
}

function compute_q_ddot(q, q_dot) {
  let J = compute_J(q);
  let J_dot = compute_J_dot(q, q_dot);
  let F_ext = compute_F_ext(q, q_dot);
  let JT = math.transpose(J);
  let zero = math.zeros(J.length, J.length);
  
  let A = math.concat(math.concat(M, JT, 1), math.concat(J, zero, 1), 0);
  let b_top = F_ext;
  let b_bottom = math.multiply(-1, math.multiply(J_dot, q_dot));
  let b = b_top.concat(b_bottom);
  let x = math.lusolve(A, b).valueOf().flat();
  return x.slice(0, 6);  // q_ddot
}

function compute_J(q) {
  let [x1, y1, th1, x2, y2, th2] = q;
  let c = Math.cos(th1), s = Math.sin(th1);
  let dx = x2 - x1, dy = y2 - y1;
  return [
    [1, 0, -l1*c, 0, 0, 0],
    [0, 1, -l1*s, 0, 0, 0],
    [-c, -s, -dx*s + dy*c, c, s, 0],
    [0, 0, 1, 0, 0, -1]
  ];
}

function compute_J_dot(q, q_dot) {
  let [x1, y1, th1, x2, y2, th2] = q;
  let [vx1, vy1, w1, vx2, vy2, w2] = q_dot;
  let c1 = Math.cos(th1), s1 = Math.sin(th1);
  let J_dot = math.zeros(4, 6)._data;
  J_dot[0][2] = l1 * s1 * w1;
  J_dot[1][2] = -l1 * c1 * w1;
  J_dot[2][0] = s1 * w1;
  J_dot[2][1] = -c1 * w1;
  J_dot[2][2] = -vx2*s1 - x2*c1*w1 + vx1*s1 + x1*c1*w1 
                + vy2*c1 - y2*s1*w1 - vy1*c1 + y1*s1*w1;
  J_dot[2][3] = -s1 * w1;
  J_dot[2][4] = c1 * w1;
  return J_dot;
}

function compute_F_ext(q, q_dot) {
  let [x1, y1, th1, x2, y2, th2] = q;
  let [vx1, vy1, w1, vx2, vy2, w2] = q_dot;
  let c1 = Math.cos(th1), s1 = Math.sin(th1);
  
  let L = math.sqrt(x2**2 + y2**2) - 2*l1 - l2 - L0
  let L_dot = (x2*vx2 + y2*vy2) / math.sqrt(x2**2 + y2**2)
  let Fsd = K*L + C*L_dot;
  let F = [0, 0, 0, 0, 0, 0];
  F[0] =  Fsd * s1;
  F[1] = -Fsd * c1 - m1 * g;
  F[3] = -Fsd * s1;
  F[4] =  Fsd * c1 - m2 * g;
  return F;
}

function drawSystem(q) {
  let [x1, y1, th1, x2, y2, th2] = q;
  
  let dx1 = l1 * Math.sin(th1);
  let dy1 = -l1 * Math.cos(th1);
  let x1s = x1 - dx1, y1s = y1 - dy1;
  let x1e = x1 + dx1, y1e = y1 + dy1;
  
  let dx2 = l2 * Math.sin(th2);
  let dy2 = -l2 * Math.cos(th2);
  let x2s = x2 - dx2, y2s = y2 - dy2;
  let x2e = x2 + dx2, y2e = y2 + dy2;
  
  stroke(0);
  strokeWeight(0.02);
  line(x1s, y1s, x1e, y1e);
  
  stroke(200, 0, 0);
  line(x2s, y2s, x2e, y2e);
}
