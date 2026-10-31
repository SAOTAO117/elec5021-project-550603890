importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(60000);
script.traceSetConsoleLevel(TraceLevel.INFO);

var root = String(java.lang.System.getProperty("user.dir")).replace(/\\/g, "/");
var server = script.getServer("DebugServer.1");
server.setConfig(root + "/Program/TMS320F28379D.ccxml");
var session = server.openSession("*", "C28xx_CPU1");
session.target.connect();
session.memory.loadProgram(root + "/Program/IterativeController.out");
session.target.runAsynch();
Thread.sleep(1000);
script.traceWrite("ITERATIVE_CONTROLLER_STARTED");
java.lang.System.exit(0);
