//todo: 
// 		right desnity, typ klar
//		more fitness options
//		more shapes
//		angle limits
//		

var b2Vec2 = Box2D.Common.Math.b2Vec2
, b2AABB = Box2D.Collision.b2AABB
, b2BodyDef = Box2D.Dynamics.b2BodyDef
, b2Body = Box2D.Dynamics.b2Body
, b2FixtureDef = Box2D.Dynamics.b2FixtureDef
, b2Fixture = Box2D.Dynamics.b2Fixture
, b2World = Box2D.Dynamics.b2World
, b2MassData = Box2D.Collision.Shapes.b2MassData
, b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
, b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
, b2DebugDraw = Box2D.Dynamics.b2DebugDraw
, b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef
, b2DistanceJointDef =  Box2D.Dynamics.Joints.b2DistanceJointDef
, b2RevoluteJointDef =  Box2D.Dynamics.Joints.b2RevoluteJointDef
, b2Shape = Box2D.Collision.Shapes.b2Shape
, b2Joint = Box2D.Dynamics.Joints.b2Joint
, b2Settings = Box2D.Common.b2Settings
, b2ContactFilter = Box2D.Dynamics.b2ContactFilter
, b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef
;

function GeneticAlgorithm(sworld, dworld, pos){
	this.simulation = new GASimulation(sworld, pos);
	this.draw  = new GADraw(dworld, pos);
	this.pos = pos;

	this.popsize;
	this.mutationstrength;
	this.mutationrate;
	this.recombinationrate;
	this.brainrecombrate;
	this.life;
	this.elitism;
	this.weights;


	this.setParams = function(params, w){
		this.popsize 		  	= params[0];
		this.mutationstrength 	= params[1];
		this.mutationrate 		= params[2];
		this.recombinationrate 	= params[3];
		this.brainrecombrate 	= params[4];
		this.life 				= params[5];
		this.elitism 			= params[6];
		// this.resistance 		= params[7];

		this.simulation.resistance = params[7];
		this.draw.resistance = params[7];

		if(this.popsize<this.elitism){
			this.elitism = this.popsize;
		}

		var sum = w.reduce(function(pv, cv) { return pv + cv; }, 0);
		for(var i=0; i<w.length; i++){
			w[i] /= sum;
		}
		this.weights = w;
	}

	this.init = function(params, w){
		this.simulation.generation = 0;
		this.draw.generation = 0;

		this.setParams(params, w);
		this.simulation.initPopulation(this.popsize);

		var self = this;
    	this.intervalNG = setInterval(function() {
        	self.nextGeneration();}, this.life*1000);
	}

	this.change = function(params, w){
		this.setParams(params, w);

		clearInterval(this.intervalNG);
		var self = this;
    	this.intervalNG = setInterval(function() {
        	self.nextGeneration();}, this.life*1000);
	}

	this.create = function(){
		this.simulation.createPoulation();
	}

	this.nextGeneration = function(){
		this.eval();
		this.simulation.destroyPopulation();
		this.simulation.generation++;
		if(this.simulation.simulate){
			this.mate();
			this.mutate();
			this.simulation.createPoulation();

			if(this.draw.auto){
				this.draw.generation = this.simulation.generation-1;
				this.draw.showGen();
			}
		}
	}

	this.eval = function(){
		var sensorChecks = this.simulation.sensorsChecked; 
		var maxStill = 0, maxProgess = 0, maxSpeedX = 0, maxSpeedY = 0, maxXpos = 0,
			maxNonContact = 0, maxAccumHeight = 0, maxYpos = 0;
		for(var i=0; i<this.simulation.population.length; i++){
			var c = this.simulation.population[i];

			var still 		= (sensorChecks - Math.abs(c.progress));
			var progress 	= (sensorChecks + c.progress);
			var speedX 		= (c.speedX/sensorChecks);
			var speedY 		= (c.speedY/sensorChecks);
			var xpos 		= (c.position().x);
			var nonContact 	= (sensorChecks - c.contact);
			var accumHeight = c.accHeight;
			var ypos  		= c.maxHeight;
 
			maxStill 		= (maxStill>still) 				? maxStill:still;
			maxProgess 		= (maxProgess>progress) 		? maxProgess:progress;
			maxSpeedX 		= (maxSpeedX>speedX) 			? maxSpeedX:speedX;
			maxSpeedY 		= (maxSpeedY>speedY) 			? maxSpeedY:speedY;
			maxXpos 		= (maxXpos>xpos) 				? maxXpos:xpos;
			maxNonContact 	= (maxNonContact>nonContact) 	? maxNonContact:nonContact;
			maxAccumHeight 	= (maxAccumHeight>accumHeight) 	? maxAccumHeight:accumHeight;
			maxYpos 		= (maxYpos>ypos) 				? maxYpos:ypos;
		}

		for(var i=0; i<this.simulation.population.length; i++){
			var c = this.simulation.population[i];
			var evalFitness = 	[(sensorChecks - Math.abs(c.progress))/maxStill,	//standing still
								 (sensorChecks + c.progress)/(maxProgess),			//progressing in x
								 ((c.speedX/sensorChecks))/(maxSpeedX),				//speed in x
								 ((c.speedY/sensorChecks))/(maxSpeedY),				//speed in y
								 (c.position().x/maxXpos),							//final x pos
							 	 (sensorChecks - c.contact)/maxNonContact,			//main body off ground
								 (c.accHeight/maxAccumHeight),						//accumulated height
								 (c.maxHeight/maxYpos)];							//Highest y pos
			c.fitness = dot(this.weights,evalFitness);
			var f = dot(this.weights, evalFitness);

		}

		this.simulation.population.sort(function(a,b){return b.fitness - a.fitness});
		var c = this.simulation.population[0];

		//print absolute fitness in console to compare and analyse result
		// console.log("Generation " + this.simulation.generation + ": ");
		// for(var i=0; i<3; i++){
			// var c = this.simulation.population[i];
			// var progressFitness = 0.5*(c.position().x/100.0)+0.5*((sensorChecks+c.progress)/(2*sensorChecks));
			// console.log("Fitness " + i +": " + progressFitness);
		// }
		// console.log("");
		this.draw.creatures.push(c.copy());

		this.simulation.sensorsChecked = 0;
	}

	this.select = function(){
		var prob = Math.random();
		var poplength = this.simulation.population.length;

		var n = 0;
		for(var i=1; i<=poplength; i++){
			n += i;
		}

		var creatureProb = 0.0;
		for(var i=0; i<poplength; i++){
			creatureProb += (poplength-i)/n;
			if(prob < creatureProb){
				return this.simulation.population[i].copy();
			}
		}
	}

	this.mate = function(){
        var newPopulation = new Array();
        for(var i=0; i<this.elitism; i++){
            newPopulation.push(this.simulation.population[i].copy());
        }
        
        while(newPopulation.length < this.popsize){
            //selection
            c1 = this.select();
            c2 = this.select();
            
            c1.recombine(c2, this.recombinationrate, this.brainrecombrate);

            c1.recombed = true;
            c2.recombed = true;

            newPopulation.push(c1);
            if(newPopulation.length < this.popsize){
                newPopulation.push(c2)
            }
        } 

        this.simulation.population.length = 0;
        this.simulation.population = newPopulation.slice();
    }

    this.mutate = function(){
		for(var i=this.elitism; i<this.simulation.population.length; i++){
			this.simulation.population[i].mutate(this.mutationrate, this.mutationstrength);
			this.simulation.population[i].mutated = true;
		}
    }

    this.showPrevGen = function(){
    	this.draw.generation--;
        if(this.draw.generation >= 0){
            this.draw.showGen(this.pos);
        }else{
            this.draw.generation++;
        }
    }

    this.showNextGen = function(){
    	this.draw.generation++;
        if(this.draw.generation <= this.simulation.generation){
            this.draw.showGen(this.pos);
        }else{
            this.draw.generation--;
        }
    }

    this.toggleSimulation = function(){
		this.simulation.simulate = !this.simulation.simulate;
		if(!this.simulation.simulate){
    		this.simulation.destroyPopulation();
    		clearInterval(this.intervalNG);
    	}else{
    		if(this.simulation.population.length){
	    		this.simulation.createPoulation();
	    		var self = this;
	    		this.intervalNG =  setInterval(function() {
	    		self.nextGeneration();}, this.life*1000);
	    	}
    	}
    }

    this.destroyAll = function(){
    	clearInterval(this.intervalNG);
    	
    	this.simulation.destroyPopulation();
    	while(this.simulation.population.length){
    		this.simulation.population.pop();
    	}
    	while(this.draw.creatures.length){
    		this.draw.creatures.pop();
    	}

    	if(this.draw.creature != null){
	    	this.draw.creature.destroy(this.draw.world);
	    	this.draw.creature = null;
	    }
    }

    this.updateResistance = function(resistance){
    	this.simulation.updateResistance(resistance);
    	this.draw.updateResistance(resistance);
    }

    this.intervalNG;
}

function GASimulation(world, pos){
	// this.p = parent; 
	this.world = world;
	this.generation = 0;
	this.population = new Array();
	this.sensorsChecked = 0;
	this.simulate = true;
	this.created = false;
	this.camera = pos;
	this.pos = pos;
	this.resistance = 0;

	this.initPopulation = function(popsize){
		this.simulate = true;
		this.generation++;
		for(var i=0; i<popsize; i++){
			var c = new Creature(i);
			c.init();
			this.population.push(c);
		}
	}

	this.createPoulation = function(){
		this.created = true;
		for(var i=0; i<this.population.length; i++){
			this.population[i].create(this.world, this.pos, this.resistance);
		}
	}

	this.destroyPopulation = function(){
		this.created = false;
		for(var i=0; i<this.population.length; i++){
			this.population[i].destroy();
		}
	}

	this.checkPopulationSensors = function(){
		if(this.simulate && this.created){
			this.sensorsChecked++;
			for(var i=0; i<this.population.length; i++){
				this.population[i].sensorCheck();
			}
		}
	}

	this.updatePopulation = function(){
		if(this.simulate && this.created){
			for(var i=0; i<this.population.length; i++){
				this.population[i].brainFunction();
			}
		}
	}

	this.updateCamera = function(){
		var pos = this.pos;
		if(this.simulate && this.created){
			for(var i=0; i<this.population.length; i++){
				var cpos = this.population[i].position();
				if(cpos.x > pos.x){
					pos = cpos;
				}
			}
		}
		this.camera = pos;
	}

	this.updateResistance = function(r){
		this.resistance = r;
		for(var i=0; i<this.population.length; i++){
			this.population[i].box2DC.updateResistance(r);
		}
	}

	var self = this;
	this.intervalPS = setInterval(function() {
        self.updatePopulation();}, 100);

	this.intervalPE = setInterval(function () {
        self.checkPopulationSensors();}, 500);

	this.intervalUC = setInterval(function () {
		self.updateCamera();}, 1000/60);
}

function GADraw(world, pos){
	this.world = world;
	this.generation = 0;
	this.auto = true;

	this.creatures = new Array();
	this.creature  = null;

	this.camera = pos;
	this.pos = pos;
	this.resistance = 0;

	this.showGen = function(){
		this.camera = this.pos;
        if(this.creature != null){
            this.creature.destroy(this.world);
            this.creature = null;
        }

        if((this.generation-1 >= 0) && (this.generation <= this.creatures.length)){
	        this.creature = this.creatures[this.generation-1];
	        this.creature.create(this.world, this.pos, this.resistance);
	    }
    }

    this.updateCreature = function(){
    	if(this.creature != null){
    		this.creature.brainFunction();
    	}
    }

    this.updateCamera = function(){
    	if(this.creature != null){
    		this.camera = this.creature.position();
    	}else{
    		this.camera = this.pos;
    	}
    }

    this.updateResistance = function(r){
    	this.resistance = r;
    	if(this.creature != null){
    		this.creature.box2DC.updateResistance(r);
    	}
    }

    var self = this;
	this.interval = setInterval(function() {
        self.updateCreature();}, 100);
	this.intervalUC = setInterval(function () {
		self.updateCamera();}, 1000/60);
}

function Creature(id, world){
	this.id = id;
	this.def;// 	= new CreatureDef();
	this.box2DC;// = new Box2DCreature(this.def);
	this.brain;//  = new Brain(2+3*this.def.numLegParts, this.def.numLegParts);

	this.fitness 	= 0.0;

	this.contact 	= 0.0;
	this.progress 	= 0.0;
	this.speedX 	= 0.0;
	this.speedY 	= 0.0;
	this.maxHeight  = 0.0;
	this.accHeight  = 0.0;

	this.lastPos;
	this.origin;


	this.init = function(){
		this.def 	= new CreatureDef();
		this.def.init();

		this.box2DC = new Box2DCreature(this.def);
		
		this.brain  = new Brain(2+3*this.def.numLegParts, this.def.numLegParts);
		this.brain.init();
	}

	this.copy = function (){
		var c 	= new Creature();
		c.id 	= this.id;
		c.def 	= this.def.copy();
		c.box2DC = new Box2DCreature(c.def);
		c.brain = this.brain.copy();

		c.fitness = 0.0;

		c.contact 	= 0.0;
		c.progress 	= 0.0;
		c.speedX 	= 0.0;
		c.speedY 	= 0.0;
		c.maxHeight = 0.0;
		c.accHeight = 0.0;

		return c;
	}	 

	this.position = function(){
		return this.box2DC.bodies[0].GetWorldCenter().Copy();
	}

	this.create = function(world, pos, r){
		this.origin = pos;
		this.box2DC.create(world, pos, r);
		this.lastPos = this.position();
	}

	this.destroy = function(){
		this.box2DC.destroy();
	}

	this.sensorCheck = function(){
		this.contact 	+= (this.box2DC.contactSensor() > 0) ? 1 : 0;
		this.progress 	+= this.box2DC.progressSensor(this.lastPos);
		this.speedX 	+= Math.abs((this.position().x-this.lastPos.x));
		this.speedY     += Math.abs((this.position().y-this.lastPos.y));

		var height = this.box2DC.heightSensor();
		this.accHeight	+= Math.abs(this.origin.y-height);// - (this.origin.y-2))/(this.origin.y-2);
		this.maxHeight  =  (this.maxHeight<height) ? this.maxHeight : height;
		// this.asccHeight	+= ((this.origin.y-2)-this.position().y);
		// this.maxHeight  =  (this.maxHeight>((this.origin.y-2)-this.position().y))? this.maxHeight : ((this.origin.y-2)-this.position().y);
		this.lastPos 	= 	this.position();
	}

	this.brainFunction = function(){
		var input = this.box2DC.brainInput();
		var output = this.brain.calculate(input, this.def, this.box2DC);
		// console.log(output);
		this.box2DC.applyTorque(output);
	}

	this.recombine = function(c, rrate, brrate){
		this.def.recombine(c.def, rrate);
		this.brain.recombine(c.brain, brrate);
	}

	this.mutate = function(mrate, mstr){
		// console.log("mutating");
		this.brain.mutate(mrate, mstr);
		this.def.mutate(mrate, mstr);

		var prob = Math.random();
		var pos = this.def.leg1.length;
		// console.log(this.def.numLegParts + " " + this.def.leg1.length + " " + this.def.leg2.length);
		if(prob < mrate){
			if(0.33 > Math.random()){
				if(this.def.numLegParts > 1 && this.def.leg1.length > 0){
					// console.log("removing shit");
					this.brain.remove(this.def, 1);
					this.def.removeLegPart(1);

				}
			}else{
				this.brain.add(this.def,1);
				this.def.addLegPart(1);
			}
		}

		prob = Math.random();
		if(prob < mrate){
			if(0.33 > Math.random()){
				if(this.def.numLegParts > 1 && this.def.leg2.length > 0){
					// console.log("removing shit");
					this.brain.remove(this.def,2);
					this.def.removeLegPart(2);
				}
			}else{
				this.brain.add(this.def,2);
				this.def.addLegPart(2);

			}
		}  
	}
}

function Brain(numInput, numOutput){
    this.numInput  = numInput;
    this.numOutput = numOutput;
    this.hidden = [];
    this.output = [];

    this.numHidden = 2*numInput;

    this.oldOutput = [];
    for(var i=0; i<this.numOutput; i++){
    	this.oldOutput.push(0);	
    }

    this.init = function(){
	    for(var i=0; i<this.numHidden; i++){
	        this.hidden[i] = [];
	        for(var j=0; j<this.numInput; j++){
	            this.hidden[i][j] = 2*Math.random()-1;
	        }
	    }


	    for(var i=0; i<this.numOutput; i++){
	        this.output[i] = [];
	        for(var j=0; j<this.numHidden; j++){
	            this.output[i][j] = 2*Math.random()-1;
	        }
	    }

	}

    this.copy = function(){
        var b = new Brain(this.numInput, this.numOutput);
        b.hidden = [];
        b.output = [];

        for(var i=0; i<this.numHidden; i++){
            b.hidden[i] = this.hidden[i].slice();
        }

        for(var i=0; i<this.numOutput; i++){
            b.output[i] = this.output[i].slice();
        }
        return b;
    }

    this.calculate = function (input, def, c){
        for(var i=0; i<this.oldOutput.length; i++){
        	input.push(this.oldOutput[i]);
        }

        var inputl = input.length;
        while(input.length > this.numInput){
        	input.pop();
        }
  		while(input.length < this.numInput){
  			input.push(0);
  		}

  		// console.log(this.numInput)
    	// console.log(input);
     	// if(input.length != this.hidden[0].length){
     		// console.log(input)
     		// console.log(def.numLegParts + " " + input.length +  " " + this.numInput +" " + this.hidden[0].length + " " + c.joints.length);
     	// }

        var hiddenOutput = [];
        for(var i=0; i<this.numHidden; i++){
            hiddenOutput[i] = transfer(dot(input,this.hidden[i]));
        }

        var output = [];
        for(var i=0; i<this.numOutput; i++){

            output[i] = transfer(dot(hiddenOutput,this.output[i]));
        }

        // console.log(output);
        this.oldOutput = output.slice();
        // console.log(this.oldOutput);
        return output;
    }

    this.changeBrain = function(nodes1, nodes2){
    	var prob = Math.random();
    	if(	(nodes1.length == nodes2.length) && (nodes1.length != 0 || nodes2.length != 0) &&
    		(nodes1[0].length == nodes2[0].length)){
    		for(var i=0; i<Math.floor(nodes1.length*Math.random()); i++){
    			var tmp = nodes1[i].slice();
    			nodes1[i] = nodes2[i].slice();
    			nodes2[i] = tmp;
    		}
    	}
    }

    this.recombine = function(b, brate){
    	var prob = Math.random();
    	if(prob < brate) this.changeBrain(this.hidden,  b.hidden );

    	prob = Math.random();
    	if(prob < brate) this.changeBrain(this.output,  b.output );
    }

    this.mutate = function(mrate, mstr){
        for(var i=0; i<this.numHidden; i++){
            for(var j=0; j<this.numInput; j++){
                var prob = Math.random();
                if(prob < /*mutation rate*/ mrate){
                    this.hidden[i][j] += 2*mstr*Math.random()-mstr;
                }
            }
        }

        for(var i=0; i<this.numOutput; i++){
            for(var j=0; j<this.numHidden; j++){
                var prob = Math.random();
                if(prob < /*mutation rate*/ mrate){
                    this.output[i][j] += 2*mstr*Math.random()-mstr;
                }
            }
        }
    }

    this.add = function(def, n){
    	var oldNumInput  = this.numInput;
    	var oldNumHidden = this.numHidden;
    	var oldNumOutput = this.numOutput;

    	this.numInput += 3;
    	this.numOutput++;// = def.numLegParts+1;
    	this.numHidden = 2*this.numInput;

    	// console.log(def.numLegParts + " " + (def.leg1.length+def.leg2.length));

    	for(var i=0; i<oldNumOutput-this.numOutput; i++){
    		this.oldOutput.push(0);
    	}
    	
    	var posc, posa;
    	if(n==1){
	    	posc = 2+def.leg1.length;
	    	posa = posc+def.leg2.length+1*def.leg1.length;
	    }else{
	    	posc = 2+def.leg1.length+def.leg2.length;
	    	posa = posc+1*(def.leg1.length+def.leg2.length);
	    }

	    var ol = this.hidden[0].length
    	for(var i=0; i<oldNumHidden; i++){
    		this.hidden[i].splice(posc,0,(2*Math.random()-1));
    		this.hidden[i].splice(posa,0,(2*Math.random()-1));//,(2*Math.random()-1));
    		this.hidden[i].push((2*Math.random()-1));
    	}

    	if(this.numInput != this.hidden[0].length){
	    		console.log("adding: " + def.numLegParts +" before: " + oldNumInput + " " +ol + " after:" + this.numInput + " " + this.hidden[0].length)
    	}


    	for(var i=oldNumHidden; i<this.numHidden; i++){
    		this.hidden[i] = [];
    		for(var j=0; j<this.numInput; j++){
    			this.hidden[i][j] = 2*Math.random()-1;
    		}
    	}

    	for(var i=0; i<oldNumOutput; i++){
    		for(var j=0; j<(this.numHidden-oldNumHidden); j++){
    			this.output[i].push(2*Math.random()-1);
    		}
    	}

	    for(var i=oldNumOutput; i<this.numOutput; i++){
	        this.output[i] = [];
	        for(var j=0; j<this.numHidden; j++){
	            this.output[i][j] = 2*Math.random()-1;
	        }
	    }
    }

    this.remove = function(def, n){
    	var oldNumInput  = this.numInput;
    	var oldNumHidden = this.numHidden;
    	var oldNumOutput = this.numOutput;
    	// if(def.numLegParts > 2){
	   		this.numInput -= 3; //2+3*(def.numLegParts-1);
	    	this.numOutput--;// = def.numLegParts-1;
	    	this.numHidden = 2*this.numInput;

	    	var posc, posa;
	    	if(n==1){
		    	posc = 1+def.leg1.length;
		    	posa = posc+def.leg2.length+def.leg1.length;
		    }else{
		    	posc = 1+def.leg1.length+def.leg2.length;
		    	posa = posc+(def.leg1.length+def.leg2.length);
		    }

		    var ol = this.hidden[0].length;
		    for(var i=0; i<this.hidden.length; i++){
	    		this.hidden[i].splice(posc,1);
	    		this.hidden[i].splice(posa,1);//,(2*Math.random()-1));
	    		this.hidden[i].pop();
	    	}

	    	for(var i=0; i<oldNumOutput; i++){
	    		for(var j=0; j<(oldNumHidden-this.numHidden); j++){
	    			this.output[i].pop();
	    		}
	    	}

		    // console.log(oldNumOutput + " " + this.numOutput + " " + this.output.length + " " + (def.numLegParts-1));
		// }
    }
}

function CreatureDef(){
	this.size = new b2Vec2(0.7,0.7);

	this.jointPos;
	this.leg1;
	this.leg2;
	this.numLegParts;


	this.init = function(){
		this.jointPos = new b2Vec2(Math.random(), Math.random());
		var s = this.size.Copy();
		s.Subtract(new b2Vec2(0.2,0.2));

		var leg = Math.floor(2*Math.random())+1;
		this.leg1 = [];
		this.leg2 = [];

		if(leg == 1){
			this.leg1.push(new LegPartDef(s));
			var p = Math.random();
			if(p < 0.2){
				this.leg2.push(new LegPartDef(s));
			}
		}else{
			this.leg2.push(new LegPartDef(s));
			var p = Math.random();
			if(p < 0.2){
				this.leg1.push(new LegPartDef(s));
			}
		}

		this.numLegParts = this.leg1.length+this.leg2.length;
	}

	this.copy = function(){
		var cdef = new CreatureDef();
		cdef.size  = this.size.Copy();
		cdef.jointPos   = this.jointPos.Copy();

		cdef.leg1 = [];
		for(var i=0; i<this.leg1.length; i++){
			cdef.leg1.push(this.leg1[i].copy());
		}
		cdef.leg2 = [];
		for(var i=0; i<this.leg2.length; i++){
			cdef.leg2.push(this.leg2[i].copy());
		}
		cdef.numLegParts = this.numLegParts;

		return cdef;
	}

	this.legLength = function(n){
		var length = 0;
		// length += this.jointPos.y;
		var leg = (n == 1) ? this.leg1 : this.leg2;
		for(var i=0; i<leg.length; i++){
			length += (leg[i].distance.y>0)? leg[i].distance.y:0;
			if(i == leg.length-1){
				length+= leg[i].size.y;
			}
		}
		return length;
	}

	this.recombine = function(def, rrate){
		var c1 = this;
		var c2 = def;
        //recombination
        c1lenght = c1.leg1.length+c1.leg2.length;
        c2lenght = c2.leg1.length+c2.leg2.length;

        if(c1lenght > c2lenght){
            var tmp = c1;
            c1 = c2;
            c2 = tmp;

            tmp = c1lenght;
            c1lenght = c2lenght;
            c2lenght = tmp;
        }

        if(Math.random() < rrate){
            var pivot = Math.floor(Math.random()*c1lenght);

            var tmpC1 = new Array();
            var tmpC2 = new Array();

            for(var i=0; i<c1.leg1.length; i++){
            	tmpC1.push(c1.leg1[i].copy());
            }
            for(var i=0; i<c1.leg2.length; i++){
            	tmpC1.push(c1.leg2[i].copy());
            }

            for(var i=0; i<c2.leg1.length; i++){
            	tmpC2.push(c2.leg1[i].copy());
            }
            for(var i=0; i<c2.leg2.length; i++){
            	tmpC2.push(c2.leg2[i].copy());
            }

            var ntmpC1 = new Array(), ntmpC2 = new Array();
            for(var i=0; i<c1lenght; i++){
                if(i<pivot){
                    ntmpC1.push(tmpC2[i]);
                    ntmpC2.push(tmpC1[i]);
                }else if(i == pivot){	
                    var legs = tmpC1[i].recombine(tmpC2[i]);
                    ntmpC1.push(legs[0]);
                    ntmpC2.push(legs[1]);
                }else{
                    ntmpC1.push(tmpC1[i]);
                    ntmpC2.push(tmpC2[i]);
                }
            }

            for(var i=c1lenght; i<c2lenght; i++){
                ntmpC2.push(tmpC2[i]);
            }

            c1l1 = c1.leg1.length, c1l2 = c1.leg2.length;
            c2l1 = c2.leg2.length, c2l2 = c2.leg2.length;

            c1.leg1 = [], c1.leg2 = [];
            c2.leg1 = [], c2.leg2 = [];

            for(var i=0; i<ntmpC1.length; i++){
                if(i<c1l1)  c1.leg1.push(ntmpC1[i]);
                else        c1.leg2.push(ntmpC1[i]);
                
            }

            for(var i=0; i<ntmpC2.length; i++){
                if(i<c2l1)  c2.leg1.push(ntmpC2[i]);
                else        c2.leg2.push(ntmpC2[i]);
            }
        }
    }
	
	this.mutate = function(mrate, mstr){
		var prob = Math.random();
		if(prob < mrate){
			var mutation = new b2Vec2(2*mstr*Math.random()-mstr, 2*mstr*Math.random()-mstr);
			this.size.Add(mutation);
		}
		prob = Math.random();
		if(prob < mrate){
			var mutation = new b2Vec2(2*mstr*Math.random()-mstr, 2*mstr*Math.random()-mstr);
			this.jointPos.Add(mutation);
		}

		if(this.size.x < 0.1) this.size.x = 0.1;
		if(this.size.x > 1.0) this.size.y = 1.0; 
		if(this.size.y < 0.1) this.size.y = 0.1;
		if(this.size.y > 1.0) this.size.y = 1.0;

		if(this.jointPos.x < 0.0) this.jointPos.x = 0.0;
		if(this.jointPos.x > this.size.x) this.jointPos.y = this.size.x; 
		if(this.jointPos.y < 0.0) this.jointPos.y = 0.0;
		if(this.jointPos.y > this.size.y) this.jointPos.y = this.size.y;

		for(var i=0; i<this.leg1.length; i++){
			this.leg1[i].mutate(mrate, mstr);
		}

		for(var i=0; i<this.leg2.length; i++){
			this.leg2[i].mutate(mrate,mstr);
		}
	}

	this.addLegPart = function(n){
		var leg = (n == 1) ? this.leg1 : this.leg2;
		
		if(leg.length == 0){
			// console.log(leg);
			// leg = new Array();
			var s = this.size.Copy();
			s.Subtract(new b2Vec2(0.2,0.2));
			leg.push(new LegPartDef(s));
		}else{
			leg.push(new LegPartDef(leg[leg.length-1].size));
		}
		// console.log(leg[leg.length-1]);
		this.numLegParts++;
	}

	this.removeLegPart = function(n){
		var leg = (n == 1) ? this.leg1 : this.leg2;

		if(leg.length > 0){
			leg.pop();
			this.numLegParts--;
		}
		// console.log("removing, " + leg.length + " " + this.numLegParts);
		// if(leg.length <= 0){
			// console.log("No more leg, " + this.numLegParts);
			// leg = [];
			// console.log(leg);

		// }
	}
}

function LegPartDef(size){
    this.size 		= new b2Vec2(size.x,size.y);
    // this.pos 		= new b2Vec2(0,0);
    this.distance  	=  new b2Vec2(0.25,0.8);
    this.strength   = (Math.random()+1);

    this.copy   = function(){
	    var legp = new LegPartDef(this.size);
	    legp.size = this.size.Copy();
	    // legp.pos  = this.pos.Copy();
	    legp.distance = this.distance.Copy();
	    legp.strength = this.strength;
	    return legp;
    }

    this.recombine = function(leg){
    	var leg1 = this.copy();
    	var leg2 = leg.copy();

    	var numgenes = 3;
    	var pivot = Math.floor(Math.random()*numgenes);
    	var keys = Object.keys(leg1);
    	for(var i=0; i<pivot; i++){
    		leg1[keys[i]] = leg[keys[i]];
    		leg2[keys[i]] = this[keys[i]];
    	}
    	return [leg1, leg2];
    }

    this.mutate = function(mrate, mstr){
    	var keys = Object.keys(this);
	    for(var i=0; i<2; i++){
	    	var prob = Math.random();
	    	if(prob < mrate){
		    	var mutation = new b2Vec2(2*mstr*Math.random()-mstr, 2*mstr*Math.random()-mstr);
		        this[keys[i]].Add(mutation);
		    }
	    }

	    if(this.size.x < 0.1) this.size.x = 0.1;
		if(this.size.x > 1.0) this.size.y = 1.0; 
		if(this.size.y < 0.1) this.size.y = 0.1;
		if(this.size.y > 1.0) this.size.y = 1.0;
	    
	    var prob = Math.random();
	    if(prob < mrate){
	    	this.strength += 2*mstr*Math.random()-mstr;
	    	if(this.strength < 0.1) this.strength = 0.1;
	    	if(this.strength > 4.0) this.strength = 4.0;
	    }
    }
}

function Box2DCreature(def){
	this.bodyDef = def;
	this.world;
	this.bodies = new Array();
	this.joints = new Array();

	this.pivots = new Array();
	this.torquePivots = new Array();

	this.jointStrengths = new Array();


	this.create = function(world,pos, r){
		this.jointStrengths = [];
		for(var i=0; i<def.leg1.length; i++){
			this.jointStrengths.push(def.leg1[i].strength);
		}
		for(var i=0; i<def.leg2.length; i++){
			this.jointStrengths.push(def.leg2[i].strength);
		}

		this.world = world;
		var fixDef  = new b2FixtureDef();
	    var bodyDef = new b2BodyDef();

	    fixDef.filter.groupIndex = -1;
	    fixDef.shape = new b2PolygonShape();
	    bodyDef.type = b2Body.b2_dynamicBody;
	    bodyDef.fixedRotation = true;

	    var leg1Lenght = def.legLength(1);
	    var leg2Lenght = def.legLength(2);
	    var lengthy = (leg1Lenght >= leg2Lenght) ? leg1Lenght : leg2Lenght;

	    var newpos = pos.Copy();
	    newpos.Add(new b2Vec2(0,-(lengthy)));
	    bodyDef.position.SetV(newpos);

	    fixDef.density = 5.0;
	    fixDef.friction = 0.2;
	    fixDef.restitution = 0.0;

	    // fixDef.shape = shapePart(this);
	    fixDef.shape = new b2PolygonShape;
	    fixDef.shape.SetAsBox(def.size.x,def.size.y);

	    var body = this.world.CreateBody(bodyDef);
	    body.SetLinearDamping(r);
	    body.CreateFixture(fixDef);
	    this.bodies.push(body);
	    this.createLeg(body, 1, r);
	    this.createLeg(body, 2, r);
	}

	this.destroy = function(){
		while(this.joints.length){
            this.world.DestroyJoint(this.joints.pop());
        }

		while(this.bodies.length){
			this.world.DestroyBody(this.bodies.pop());
		}

        while(this.pivots.length){
        	this.world.DestroyBody(this.pivots.pop());
        }

        while(this.torquePivots.length){
        	this.world.DestroyBody(this.torquePivots.pop());
        }
	}

	this.createLeg = function(curr, n, r){
		var fixDef  = new b2FixtureDef();
        var bodyDef = new b2BodyDef();
        var weldDef = new b2WeldJointDef();
        var revDef  = new b2RevoluteJointDef();

        fixDef.density = 4.0;
        fixDef.friction = 2.0;
        fixDef.restitution = 0.0;

        fixDef.filter.groupIndex = -1;
        bodyDef.type = b2Body.b2_dynamicBody;

        var legDefs = (n == 1) ? this.bodyDef.leg1 : this.bodyDef.leg2;
        var pos;
        if(n == 1){
        	pos = new b2Vec2(this.bodyDef.jointPos.x*this.bodyDef.size.x,
        	 				 this.bodyDef.jointPos.y*this.bodyDef.size.y);
        }else{
        	pos = new b2Vec2(-this.bodyDef.jointPos.x*this.bodyDef.size.x, 
        					  this.bodyDef.jointPos.y*this.bodyDef.size.y);
        }

        var next;
        for(var i=0; i<legDefs.length; i++){
        	var distance;
        	if(n == 1){
        		distance = legDefs[i].distance;
        	}else{
        		distance = new b2Vec2(-legDefs[i].distance.x, legDefs[i].distance.y);
        	}

	        fixDef.shape = new b2PolygonShape;
	        fixDef.shape.SetAsBox(legDefs[i].size.x,legDefs[i].size.y);
	        bodyDef.position.SetV(curr.GetWorldCenter());
	        bodyDef.position.Add(distance);

	        if(i != (legDefs.length-1)){
	        	bodyDef.fixedRotation = true;
	        }

	        var next = this.world.CreateBody(bodyDef);
	        next.SetLinearDamping(r);
        	next.CreateFixture(fixDef);

        	bodyDef.fixedRotation = false;
        	fixDef.density = 2.0;
	        fixDef.shape = new b2CircleShape(0.1);
	        bodyDef.position.SetV(curr.GetWorldPoint(new b2Vec2(0,0)));

	        if(i==0){
	            bodyDef.position.Add(pos);
		    }

        	var pivotCurr = this.world.CreateBody(bodyDef);
	        var torquePivot = this.world.CreateBody(bodyDef);
	        pivotCurr.CreateFixture(fixDef);
	        torquePivot.CreateFixture(fixDef);

	        bodyDef.position = next.GetWorldCenter();
	        var pivotNext = this.world.CreateBody(bodyDef);
	        pivotNext.CreateFixture(fixDef);

	        var weldDef = new b2WeldJointDef();
	        weldDef.Initialize(curr, pivotCurr, curr.GetWorldCenter());
	        var weldJoint = this.world.CreateJoint(weldDef);

	        var revDef = new b2RevoluteJointDef();
	        revDef.Initialize(pivotCurr,torquePivot, pivotCurr.GetWorldCenter());
	        var joint = this.world.CreateJoint(revDef);
	        joint.SetLimits(-Math.PI/2,Math.PI/2);

	        weldDef.Initialize(torquePivot, pivotNext, torquePivot.GetWorldCenter());
	        this.world.CreateJoint(weldDef);

		    revDef.Initialize(next, pivotNext, next.GetWorldCenter());
		    var jointNext = this.world.CreateJoint(revDef);
		    jointNext.SetLimits(-Math.PI/2,Math.PI/2);

		    curr = next;

	        this.bodies.push(next);
	        this.joints.push(joint);
	        this.pivots.push(pivotCurr);
	        this.pivots.push(pivotNext);
	        this.torquePivots.push(torquePivot);
	    }
	}

	this.contactSensor = function(){
		return checkContact(this.bodies[0]);
	}

	this.progressSensor = function(lastPos){
		var x = this.pos().x;
		if(x > lastPos.x){
			return 1;
		}else if(x < lastPos.x){
			return -1;
		}
		return 0;
	}

	this.heightSensor = function(){
		var h = this.pos().y;
		for(var i=1; i<this.bodies.length; i++){
			var bh = this.bodies[i].GetPosition().y;
			if(bh>h) h = bh;
		}
		return h;
	}

	this.pos = function(){
		return this.bodies[0].GetWorldCenter().Copy();
	}

	this.brainInput = function(){
		var input = [1];
		for(var i=0; i<this.bodies.length; i++){
			input.push(checkContact(this.bodies[0]))
		}

		for(var i=0; i<this.joints.length; i++){
			input.push(Math.cos(this.joints[i].GetJointAngle()));
		}
		return input;
	}

	this.applyTorque = function(output){
		while(output.length < this.torquePivots.length){
			output.push(0); 
		}
		for(var i=0; i<this.torquePivots.length; i++){
			this.torquePivots[i].ApplyTorque(50*this.jointStrengths[i]*output[i]);
		}
	}

	this.updateResistance = function(r){
		for(var i=0; i<this.bodies.length; i++){
			this.bodies[i].SetLinearDamping(r);
		}
	}
}

function checkContact(body){
    var cl = body.GetContactList();
    var contact = false;
    if(cl != null) contact = cl.contact.IsTouching();
    return (contact>0) ? 1:-1;
}

function shapePart(shapeDef){
    if(shapeDef.shape == 0){
        shape = new b2PolygonShape;
        shape.SetAsBox(shapeDef.size.x,shapeDef.size.y);
    }else if(shapeDef.shape == 1){
        shape = new b2PolygonShape;
        var shapePoints = [{x: 0, y:0},
                              {x: 2*shapeDef.size.x, y:0},
                              {x: 0, y:2*shapeDef.size.y}];
        var points  = []
        for(var i=0; i<shapePoints.length; i++){
            var vec = new b2Vec2();
            vec.Set(shapePoints[i].x, shapePoints[i].y);
            points[i] = vec;
        }
        shape.SetAsArray(points,points.length);
    }else if(shapeDef.shape == 2){
        shape = new b2CircleShape(shapeDef.size.x/2.0);
    }
    return shape;
}

function dot(x,y){
    var dotProduct = 0;
    for(var i=0; i<x.length; i++){
        dotProduct += x[i]*y[i];
    }
    return dotProduct;
}

function transfer(x){
    var px = Math.exp(x);
    var mp = Math.exp(-x);
    return (px-mp)/(px+mp); 
    // return 1/(1+mp);
}
