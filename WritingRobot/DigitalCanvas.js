let mouseVals = [];

function setup() {
  createCanvas(400, 400);
  background(0);
  saveButton = createButton("Save to txt file")
  saveButton.mousePressed(saveFile)
}

function draw() {
  if (mouseIsPressed === true) {
    stroke(255);
    line(mouseX, mouseY, pmouseX, pmouseY);
    mouseVals.push([pmouseX, pmouseY]);
  }
}

function saveFile () {
  save(mouseVals , 'mouseValues.txt' , true)
}