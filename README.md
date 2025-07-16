# IsaacSimJackalMazeEscape
Program to use with inside of Isaac Sim with the Jackal robot provided by Isaac Sim. The robot uses lidar detection to map out the maze and then escape it using A*.

All you should need to do is to grab the usd file and open it in Isaac Sim and click play.

Isaac Sim might throw errors about the usd file. If so simly import the Jackal. It should already be in Isaac Sim. Then you can use a maze generator to make a maze for you and build it in Isaac Sim. I used https://www.multimazegenerator.com/ but it doesn't matter what you use. 

If you are going to build your maze keep in mind that the intersections on grid is where the jackal travels along. You should place your walls here and the walls should be thin enough for the jackal to detect them. The reason for this is because of the way I programmed it. Although I could have made it better I didn't have time to, so I encourage that you change the code.

Please note the usd files may not work due to Isaac Sim changing constantly. If this is the case you should be able to simply put the code into the Isaac Sim's script node.
The methods in the code may also not work if Isaac Sim has changed and you will have to find the equivalent functions.
