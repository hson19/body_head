% @doc body_head public API.
-module(body_head).

-behavior(application).

% Callbacks
-export([start/2]).
-export([stop/1,stop_all/0]).
-export([launch/0,launch_all/0]).
-export([set_args/1]).

%--- API -----------------------------------------------------------------------
set_args(nav3) ->
    Cn = nav3:calibrate(),
    R0 = kalman_filter:calibrate(element(3, Cn)),
    update_table({{nav3, node()}, Cn}),
    update_table({{kalman_filter, node()}, R0}).

launch() ->
    try launch(nil) of
        ok ->
            [grisp_led:color(L, green) || L <- [1, 2]],
            ok
    catch
        error:badarg ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            {error, badarg}
    end.

launch_all() ->
    rpc:multicall(?MODULE, launch, []).

stop_all() ->
    _ = rpc:multicall(application, stop, [hera]),
    _ = rpc:multicall(application, start, [hera]),
    ok.
%--- Callbacks -----------------------------------------------------------------

% @private
start(_Type, _Args) -> 
    {ok,Supervisor}=body_head_sup:start_link(),
    init_table(),
    _ = grisp:add_device(spi2,pmod_nav),
    io:format("Contents of ETS table args: ~p~n", [ets:tab2list(args)]),
    _ = launch(),
    {ok,Supervisor}.

% @private
stop(_State) -> 
    io:format("Stopping body_head application~n"),
    ok.



%--- Internal functions ---------------------------------------------------------
launch(_) ->
    Cn = ets:lookup_element(args,{nav3,node()},2), %renvoie le 2 ième élemet de la liste args avec la clé {nav3,node()} ce qui a été mis dans set_argss
    R0 = ets:lookup_element(args, {kalman_filter, node()}, 2),
    {ok,_} = hera:start_measure(nav3,Cn),
    {ok,_} = hera:start_measure(kalman_filter,R0),
    ok.

init_table() ->
    args = ets:new(args, [public, named_table]), % public means any process can read or write the table
    {ResL,_} = rpc:multicall(nodes(), ets, tab2list, [args]),
    L = lists:filter(fun(Res) ->
        case Res of {badrpc,_} -> false; _ -> true end end, ResL),
    lists:foreach(fun(Object) -> ets:insert(args, Object) end, L).


update_table(Object) ->
    _ = rpc:multicall(ets, insert, [args, Object]),
    ok.