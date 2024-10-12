// Download button logic
// const downloadButton = document.getElementById('downloadButton');
// const generatedCode = `{{ completion|e }}`; // Escaping completion text
// var promptTextarea = document.getElementById('promptTextarea');
// var promptEditor = CodeMirror.fromTextArea(promptTextarea, {
//     mode: 'python',
//     theme: 'default',
//     lineNumbers: true
// });

// downloadButton.addEventListener('click', () => {
//     text = promptEditor.getValue() + document.getElementById('generatedCode').innerText
//     const blob = new Blob([text], { type: 'text/plain' });
//     const url = URL.createObjectURL(blob);
//     downloadButton.href = url;
// });

// function handleFileUpload(input) {
// const file = input.files[0];
// if (file) {
//     const reader = new FileReader();
//     reader.onload = function(e) {
//         // document.getElementById('promptTextarea').value = e.target.result;
//         promptEditor.setValue(e.target.result);
//     };
//     reader.readAsText(file);
//     }
// }

function propagateChat(data, windowName) {
    for (var i = 0; i < data.messages.length; i++) {
        // $("#chat-window").append("<div><span class='llm-label'> LLM:</span> " + data.messages[i] + "</div>");
        var id = data.messages[i].role
        var content = data.messages[i].content
        console.log("ID:", id);
        console.log("Content:", content);
        // window = document.getElementById(windowName);
        // get the chat window by id in jquery
        window = $("#" + windowName);
        if (data.messages[i].type === "text") {
            // $("#chat-window").append(`<div class='label ${id}'> ${id.toUpperCase()}:</div> <div>${content}</div>`);
            $("#" + windowName).append(`<div class='label ${id}'> ${id.toUpperCase()}:</div> <div>${content}</div>`);
        } else if (data.messages[i].type === "image") {
            $("#" + windowName).append(`<div class='label ${id}'> ${id.toUpperCase()}:</div> <div> <img src='data:image/jpg;base64,${content}'></div>`);
        } else if (data.messages[i].type === "code") {
            $("#" + windowName).append(`<div class='label ${id}'> ${id.toUpperCase()}:</div> <div> <pre><code class='python'> ${content} </code></pre></div>`);
            hljs.highlightAll();
        }
    }
    $("#" + windowName).append("<hr>");
}

$(document).ready(function() {
    $("#generate-button").click(function() {
        event.preventDefault();
        $.ajax({
            type: "POST",
            url: "/generate",
            dataType: "json",
            contentType: "application/json; charset=utf-8",
            data: JSON.stringify(promptEditor.getValue()),
            success: function(data) {
                // console.log(data.completion)
                $("#generatedCode").text(data.completion);
                var codeElement = document.getElementById('generatedCode');
                // Add code mirror class for coloring (default is the theme)
                codeElement.classList.add( 'cm-s-default' );
                var code = codeElement.innerText;

                codeElement.innerHTML = "";

                CodeMirror.runMode(
                code,
                'python',
                codeElement
                );

            }
        });
    });

    $("#configure-form").submit(function(event) {
        event.preventDefault();

        var policyconfValues = $("input[name='policyconf']:checked").map(function(){
            return $(this).val();
        }).get();
        console.log("PolicyConf:", policyconfValues);
        var formData = {
            moveconf: $("input[name='moveconf']:checked").val(),
            graspconf: $("input[name='graspconf']:checked").val(),
            // policyconf: $("input[name='policyconf']:checked").val(),
            policyconf: policyconfValues,
            llmconf: $("input[name='llmconf']:checked").val(),
            vlmconf: $("input[name='vlmconf']:checked").val(),
            vlaconf: $("input[name='vlaconf']:checked").val(),
        };
        $("#connect-status").text("Configuring & Connecting");
        $("#connect-status").css("color", "gray");

        $.ajax({
            type: "POST",
            url: "/connect",
            data: JSON.stringify(formData),
            contentType: "application/json; charset=utf-8",
            dataType: "json",
            success: function(data) {
                console.log("Success:", data);
                var connectStatus = data.connected;
                if (connectStatus) {
                    console.log("Connected:", connectStatus)
                    $("#connect-status").text("Connected");
                    $("#connect-status").css("color", "green");
                    propagateChat(data, "robot-chat-window");
                }
            },
            error: function(jqXHR, textStatus, errorThrown) {
                console.log("Error:", textStatus, errorThrown);
            }
        });
    });

    $("#send-button").click(function() {
        sendMessage();
    });

    $("#new-interaction").click(function() {
        $("#chat-window").empty(); // Clear chat window
        $.ajax({
            type: "POST",
            url: "/new_interaction",
            contentType: "application/json; charset=utf-8",
            dataType: "json",
            success: function() {
                $("#chat-status").text("Saved Chat Log")
                $("#chat-status").css("color", "green")
            },
            error: function(jqXHR, textStatus, errorThrown) {
                console.log("Error:", textStatus, errorThrown);
            }
        });
    });

    $("#clear-chat").click(function() {
        $("#chat-window").empty(); // Clear chat window
    });

    $("#user-input").keypress(function(event) {
        if (event.which == 13) { // Enter key
            sendMessage();
        }
    });

    function sendMessage() {
        var message = $("#user-input").val();
        if (message === "") {
            return;
        }
        $("#chat-window").append("<div> <span class='label user'>You: </span> " + message + "</div> <hr>");
        $("#user-input").val(""); // Clear input field after sending
        $("#chat-status").text("Awaiting");
        $("#chat-status").css("color", "gray");
        $.ajax({
            type: "POST",
            url: "/chat",
            data: JSON.stringify({ message: message }),
            contentType: "application/json; charset=utf-8",
            dataType: "json",
            success: function(data) {
                $("#chat-status").text("Received");
                $("#chat-status").css("color", "green");
                console.log("Response:", data);
                propagateChat(data, "chat-window");

                // nested post
                $.ajax({
                    type: "POST",
                    url: "/grasp_policy",
                    data: JSON.stringify({ message: message }),
                    contentType: "application/json; charset=utf-8",
                    dataType: "json",
                    success: function(data) {
                        $("#chat-status").text("Grasp Policy");
                        $("#chat-status").css("color", "blue");
                        console.log("Response:", data);
                        propagateChat(data, "chat-window");
                    }
                });
                // end nested post
            },
            error: function(jqXHR, textStatus, errorThrown) {
                console.log("Error:", textStatus, errorThrown);
            }
        });
    }

    $("#robot-execute").click(function() {
        $("#robot-status").text("Awaiting Execution");
        $("#robot-status").css("color", "gray");
        $.ajax({
            type: "POST",
            url: "/execute",
            success: function(data) {
                console.log("Execute:", data);
                $("#robot-status").text("Executed");
                $("#robot-status").css("color", "green");
                propagateChat(data, "chat-window");
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-home").click(function() {
        $.ajax({
            type: "POST",
            url: "/home",
            success: function(data) {
                console.log("Home:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-keep-policy").click(function() {
        // if button is green, change to brown
        if ($("#robot-keep-policy").css("background-color") == "rgb(0, 128, 0)") {
            $("#robot-keep-policy").css("background-color", "brown");
        }
        else {
            $("#robot-keep-policy").css("background-color", "green");
        }
        $.ajax({
            type: "POST",
            url: "/keep_policy",
            success: function(data) {
                console.log("Home:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-teach-mode").click(function() {
        // if button is green, change to brown
        if ($("#robot-teach-mode").css("background-color") == "rgb(0, 128, 0)") {
            $("#robot-teach-mode").css("background-color", "brown");
        }
        else {
            $("#robot-teach-mode").css("background-color", "green");
        }
        $.ajax({
            type: "POST",
            url: "/teach_mode",
            success: function(data) {
                console.log("Home:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-set-home").click(function() {
        $.ajax({
            type: "POST",
            url: "/set_home",
            success: function(data) {
                console.log("Home:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-move").click(function() {
        $.ajax({
            type: "POST",
            url: "/move",
            success: function(data) {
                console.log("Move:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-grasp").click(function() {
        $.ajax({
            type: "POST",
            url: "/grasp",
            success: function(data) {
                console.log("Grasp:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#robot-release").click(function() {
        $.ajax({
            type: "POST",
            url: "/release",
            success: function(data) {
                console.log("Release:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-observe").click(function() {
        $.ajax({
            type: "POST",
            url: "/vla_obs",
            success: function(data) {
                console.log("Home:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-act").click(function() {
        $.ajax({
            type: "POST",
            url: "/vla_act",
            success: function(data) {
                console.log("VLA Act:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-reset").click(function() {
        $.ajax({
            type: "POST",
            url: "/vla_reset_policy",
            success: function(data) {
                console.log("VLA Reset:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-robot-toggle").click(function() {
        // if button is green, change to brown
        if ($("#vla-robot-toggle").css("background-color") == "rgb(0, 128, 0)") {
            $("#vla-robot-toggle").css("background-color", "brown");
        }
        else {
            $("#vla-robot-toggle").css("background-color", "green");
        }
        $.ajax({
            type: "POST",
            url: "/vla_robot_toggle",
            success: function(data) {
                console.log("VLA Robot Toggle:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-record-load-toggle").click(function() {
        if ($("#vla-record-load-toggle").css("background-color") == "rgb(0, 128, 0)") {
            $("#vla-record-load-toggle").css("background-color", "brown");
        }
        else {
            $("#vla-record-load-toggle").css("background-color", "green");
        }
        $.ajax({
            type: "POST",
            url: "/vla_record_load",
            success: function(data) {
                console.log("VLA Record Load Toggle:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-send-obs-act").click(function() {
        var message = $("#vla-obs-act").val();
        if (message === "") {
            return;
        }
        $.ajax({
            type: "POST",
            url: "/vla_obs_act",
            data: JSON.stringify({ message: message }),
            contentType: "application/json; charset=utf-8",
            success: function(data) {
                console.log("VLA Send Obs Action:", data);
                propagateChat(data, "robot-chat-window");
            }
        });
    });

    $("#vla-obs-act").keypress(function(event) {
        if (event.which == 13) { // Enter key
            var message = $("#vla-obs-act").val();
            if (message === "") {
                return;
            }
            $.ajax({
                type: "POST",
                url: "/vla_obs_act",
                data: JSON.stringify({ message: message }),
                contentType: "application/json; charset=utf-8",
                success: function(data) {
                    console.log("VLA Send Obs Action:", data);
                    propagateChat(data, "robot-chat-window");
                }
            });

        }
    });

});