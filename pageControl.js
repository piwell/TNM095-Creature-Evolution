var genAlg, simworld, drawworld, pos;
var showSim = true;
var followY = false;
function init() {
        Math.seedrandom();
        simworld =  new b2World(new b2Vec2(0,10),true);
        drawworld = new b2World(new b2Vec2(0,10),true);
        pos = new b2Vec2(8,13);

        genAlg = new GeneticAlgorithm(simworld, drawworld, pos);

        createGround(0);

        var simCanvas = document.getElementById("simCanvas");
        var drwCanvas = document.getElementById("drwCanvas");

        // var genSimCanvas = document.getElementById("simGenerationDisplay");
        // var genDrwCanvas = document.getElementById("drawGenerationDisplay");
        // var highscoreCanvas = document.getElementById("highscore");

        var debugDraw = new b2DebugDraw();
        debugDraw.SetSprite(simCanvas.getContext("2d"));
        debugDraw.SetDrawScale(30.0);
        debugDraw.SetFillAlpha(0.5);
        debugDraw.SetLineThickness(1.0);
        debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);

        var debugDraw2 = new b2DebugDraw();
        debugDraw2.SetSprite(drwCanvas.getContext("2d"));
        debugDraw2.SetDrawScale(30.0);
        debugDraw2.SetFillAlpha(0.5);
        debugDraw2.SetLineThickness(1.0);
        debugDraw2.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);

        simworld.SetDebugDraw(debugDraw);
        drawworld.SetDebugDraw(debugDraw2);

        window.setInterval(function(){update(simworld ,simCanvas, genAlg.simulation.camera, showSim); },  1000 / 60);
        window.setInterval(function(){update(drawworld,drwCanvas, genAlg.draw.camera, true);},      1000 / 60);
        // window.setInterval(function(){updateHighscore(highscoreCanvas, genAlg.highscore);}, 1000/60);

        document.addEventListener('keydown', function(event) {
          if(event.keyCode == 81) {
            var checked = document.getElementById("showSim").checked;
            document.getElementById("showSim").checked = !checked;
            toggleShowSim();
          }else if(event.keyCode == 87){
            var checked = document.getElementById("simulate").checked;
            document.getElementById("simulate").checked = !checked;
            toggleSim();    
          }else if(event.keyCode == 69){
            var checked = document.getElementById("auto").checked;
            document.getElementById("auto").checked = !checked;
            toggleAuto();
          }else if(event.keyCode == 89){
            var checked = document.getElementById("followy").checked;
            document.getElementById("followy").checked = !checked;
            toggleFollowY();
          }else if(event.keyCode == 65){
            genAlg.showPrevGen()
          }else if(event.keyCode == 83){
            genAlg.showNextGen();
          }else if(event.keyCode == 78){
              startSimulation();
          }
          else if(event.keyCode == 77){
              change();
          }
        });
}


	function createGround(angle){
  var createPopAgain = (genAlg.simulation.population>0);
  var createCreatureAgain = (genAlg.draw.creature != null);
  // genAlg.simulation.destroyPopulation();
  genAlg.toggleSimulation();
  if(genAlg.draw.creature != null){
      genAlg.draw.creature.destroy(genAlg.draw.world);
      genAlg.draw.creature = null;
  }

  sb = simworld.GetBodyList();
  db = drawworld.GetBodyList();

  while(sb){
    var b = sb.GetNext();
    simworld.DestroyBody(sb);
    sb = b;
  }

  while(db){
    var b = db.GetNext();
    drawworld.DestroyBody(db);
    db = b;
  }

  //create ground
  var fixDef = new b2FixtureDef;
  fixDef.density = 1.0;
  fixDef.friction = 2.0;
  fixDef.restitution = 0.2;
  
  var bodyDef = new b2BodyDef;        
  bodyDef.type = b2Body.b2_staticBody;
  fixDef.shape = new b2PolygonShape;
  fixDef.shape.SetAsBox(0.5, 0.2);
  fixDef.filter.groupIndex = 1;

  // bodyDef.position.y = pos.y+0.2;
  // bodyDef.position.x = pos.x;
  var my = py = pos.y+0.2;
  var mx = px = pos.x; 

  for(var i=0; i<1000; i++){
    // bodyDef.position.x = i;
      bodyDef.position.x = px;
      bodyDef.position.y = py;
      var ground1p  = simworld.CreateBody(bodyDef);
      var ground2p  = drawworld.CreateBody(bodyDef);
      ground1p.CreateFixture(fixDef);
      ground2p.CreateFixture(fixDef);
      // angle = 0;

      bodyDef.position.x = mx;
      bodyDef.position.y = my;
      var ground1m  = simworld.CreateBody(bodyDef);
      var ground2m  = drawworld.CreateBody(bodyDef);
      ground1m.CreateFixture(fixDef);
      ground2m.CreateFixture(fixDef);
      
      var anglep = anglem = 0.0;
      
      if(i>2){
        anglep = 2*angle*Math.random()-angle;
        anglem = 2*angle*Math.random()-angle;
        ground1p.SetAngle(anglep);
        ground2p.SetAngle(anglep);

        ground1m.SetAngle(anglem);
        ground2m.SetAngle(anglem);
      }

        px += Math.cos(anglep);
        py += Math.sin(anglep);

        mx -= Math.cos(anglem);
        my -= Math.sin(anglem);
    }

  // if(createPopAgain && genAlg.simulation.simulate) 
  genAlg.toggleSimulation()
    // genAlg.simulation.createPoulation();
  if(createCreatureAgain)
    genAlg.draw.showGen();
}

function update(world,canvas, camera, show){
    world.Step(
        1 / 60      //frame-rate
        ,  15       //velocity iterations
        ,  15       //position iterations
    );

    var ctx = canvas.getContext("2d")
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    
    if(followY){
      ctx.translate(-camera.x*30+canvas.width/2, -(camera.y-pos.y/2+2)*30+canvas.height/2-10);
    }else{
      ctx.translate(-camera.x*30+canvas.width/2, -5);
    }

    if(show == true) world.DrawDebugData();
    world.ClearForces();
    ctx.restore();

}

function updateGenerationDisplay(canvas, txt, g){
    document.querySelector('#genDisp #showGen').innerHTML = genAlg.draw.generation;
    document.querySelector('#genDisp #simGen').innerHTML =  genAlg.simulation.generation;
}

function updateTerrainValue(val){
  createGround(val);
}

function updateGravityValue(val){
		 simworld.SetGravity(new b2Vec2(0.0,val));
		drawworld.SetGravity(new b2Vec2(0.0,val));
}

function toggleShowSim(val){
  showSim = !showSim;
}

function getParams(){
    param = [
              parseFloat(document.getElementById("numPop").value),
              parseFloat(document.getElementById("mstr").value)/100.0,
              parseFloat(document.getElementById("mrate").value)/100.0,
              parseFloat(document.getElementById("rrate").value)/100.0,
              parseFloat(document.getElementById("brrate").value)/100.0,
              parseFloat(document.getElementById("life").value),
              parseFloat(document.getElementById("elit").value)
            ];
    return param;
}

function getWeights(){
  var w = [
              parseFloat(document.getElementById("w1").value),
              parseFloat(document.getElementById("w2").value),
              parseFloat(document.getElementById("w3").value),
              parseFloat(document.getElementById("w4").value),
              parseFloat(document.getElementById("w5").value),
              parseFloat(document.getElementById("w6").value),
              parseFloat(document.getElementById("w7").value),
              parseFloat(document.getElementById("w8").value)
  ];
  return w;
}

function startSimulation(){
    document.getElementById("simulate").checked = true;
    document.getElementById("start").value = "Restart! (n)";
    genAlg.destroyAll();

    var p = getParams();
    var w = getWeights();      

    genAlg.init(p, w);
    genAlg.create(pos);
    
    window.setInterval(function(){updateGenerationDisplay();}, 1000/60);
}

function change(){
  var p = getParams();
  var w = getWeights();
  genAlg.change(p,w);
}

function toggleFollowY(){
  followY = !followY; 
}

function toggleAuto(){
  genAlg.draw.auto = !genAlg.draw.auto; 
}

function toggleSim(){
  genAlg.toggleSimulation();
}