from unittest import TestCase

import asyncio
from contextlib import AsyncExitStack
from typing import Optional
import os

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from anthropic import Anthropic
from dotenv import load_dotenv


load_dotenv()


class McpServerTestBase(TestCase):
    '''
    I am not good at async programming,
    don't laugh at my stupid codes. QAQ.
    btw, this code is almostly copied from the official mcp client example.
    So I don't want to comment it, NOT BECAUSE I'M LAZY!
    '''
    async def connect_to_server(self):
        self.session: Optional[ClientSession] = None
        self.exit_stack = AsyncExitStack()
        self.anthropic = Anthropic()
        server_params = StdioServerParameters(
            command="uv",
            args=[
                "--directory",
                "/home/ros/mcp_ros2/mcp_ros2/",
                "run",
                "mcp_server.py"
            ],
            env = {
                "SHELL": os.environ["SHELL"],
                "PWD": os.environ["PWD"],
                "LOGNAME": os.environ["LOGNAME"],
                "HOME": os.environ["HOME"],
                "USERNAME": os.environ["USERNAME"],
                "PYTHONPATH": os.environ["PYTHONPATH"],
                "PATH": os.environ["PATH"],
                "USER": os.environ["USER"],
                "LD_LIBRARY_PATH": os.environ["LD_LIBRARY_PATH"],
            },
        )
        stdio_transport = await self.exit_stack.enter_async_context(stdio_client(server_params))
        self.stdio, self.write = stdio_transport
        self.session = await self.exit_stack.enter_async_context(ClientSession(self.stdio, self.write))
        
        await self.session.initialize()
        
        # List available tools
        response = await self.session.list_tools()
        tools = response.tools
        print("\nConnected to server with tools:", [tool.name for tool in tools])


    async def run_tools(self, tool_name: str, tool_args: dict):
        try:
            await self.connect_to_server()
            response = await self.session.call_tool(tool_name, tool_args)
            print(response.content)
        finally:
            await self.exit_stack.aclose()


    def test_locate_bot(self):
        asyncio.run(self.run_tools("locate_bot", {}))
    
    def test_move_forward(self):
        asyncio.run(self.run_tools("move", {"linear_x": 0.5, "angular_z": 0.0, "seconds": 1}))

    def test_move_backward(self):
        asyncio.run(self.run_tools("move", {"linear_x": -0.5, "angular_z": 0.0, "seconds": 1}))
    
    def test_save_image(self):
        asyncio.run(self.run_tools("save_image", {"path": "./images/test.png"}))
    
        